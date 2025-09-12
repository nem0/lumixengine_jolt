#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/PhysicsScene.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Renderer/DebugRendererSimple.h>
#include <Jolt/RegisterTypes.h>

#include "core/hash_map.h"
#include "core/job_system.h"
#include "core/log.h"
#include "core/os.h"
#include "core/profiler.h"
#include "core/stream.h"
#include "core/string.h"
#include "engine/engine.h"
#include "engine/plugin.h"
#include "engine/resource_manager.h"
#include "engine/reflection.h"
#include "engine/world.h"
#include "imgui/IconsFontAwesome5.h"
#include "jolt_module.h"
#include "renderer/model.h"
#include "renderer/render_module.h"

namespace Lumix {

enum class JoltVersion : i32 {
	MESHES,
	BODY_PROPERTIES,

	LATEST
};

static const ComponentType JOLT_BODY_TYPE = reflection::getComponentType("jolt_body");
static const ComponentType JOLT_BOX_TYPE = reflection::getComponentType("jolt_box");
static const ComponentType JOLT_MESH_TYPE = reflection::getComponentType("jolt_mesh");
static const ComponentType JOLT_SPHERE_TYPE = reflection::getComponentType("jolt_sphere");

namespace BroadPhaseLayers
{
	static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
	static constexpr JPH::BroadPhaseLayer MOVING(1);
	static constexpr u32 NUM_LAYERS(2);
};

namespace Layers
{
	static constexpr JPH::ObjectLayer NON_MOVING = 0;
	static constexpr JPH::ObjectLayer MOVING = 1;
	static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
};

static JPH::Vec3 toJPH(const Vec3& v) {
	return {v.x, v.y, v.z};
}

static JPH::Quat toJPH(const Quat& v) {
	return {v.x, v.y, v.z, v.w};
}

static Vec3 toLumix(const JPH::Vec3& v) {
	return {v.GetX(), v.GetY(), v.GetZ()};
}

static Color toLumix(const JPH::ColorArg& v) {
	return Color(v.r, v.g, v.b, v.a);
}

static Quat toLumix(const JPH::Quat& v) {
	return {v.GetX(), v.GetY(), v.GetZ(), v.GetW()};
}

struct BPLayerInterfaceImpl final : JPH::BroadPhaseLayerInterface {
	BPLayerInterfaceImpl() {
		mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
		mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
	}

	virtual u32 GetNumBroadPhaseLayers() const override { return BroadPhaseLayers::NUM_LAYERS; }

	virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
		ASSERT(inLayer < Layers::NUM_LAYERS);
		return mObjectToBroadPhase[inLayer];
	}

	JPH::BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

struct ObjectVsBroadPhaseLayerFilterImpl final : JPH::ObjectVsBroadPhaseLayerFilter {
	bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override {
		switch (inLayer1) {
			case Layers::NON_MOVING: return inLayer2 == BroadPhaseLayers::MOVING;
			case Layers::MOVING: return true;
			default: ASSERT(false); return false;
		}
	}
};

struct ObjectLayerPairFilterImpl final : JPH::ObjectLayerPairFilter {
	bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override {
		switch (inObject1) {
			case Layers::NON_MOVING: return inObject2 == Layers::MOVING;
			case Layers::MOVING: return true;
			default: ASSERT(false); return false;
		}
	}
};


struct JoltDebugRenderer : public JPH::DebugRendererSimple {
	void DrawLine(JPH::RVec3Arg from, JPH::RVec3Arg to, JPH::ColorArg color) override {
		if (module) module->addDebugLine(DVec3(toLumix(from)), DVec3(toLumix(to)), toLumix(color));
	}

	void DrawTriangle(JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3, JPH::ColorArg inColor, ECastShadow inCastShadow) override {
	}
	
	void DrawText3D(JPH::RVec3Arg inPosition, const std::string_view &inString, JPH::ColorArg inColor = JPH::Color::sWhite, float inHeight = 0.5f) override {
	}

	RenderModule* module = nullptr;
};


struct JoltJobSystemProxy : JPH::JobSystemWithBarrier {
	JoltJobSystemProxy() {
		Init(1024);
	}

	struct MyJob : Job {
		MyJob(const char *inJobName, JPH::ColorArg inColor, JobSystem *inJobSystem, const JobFunction &inJobFunction, JPH::uint32 inNumDependencies) 
			: Job(inJobName, inColor, inJobSystem, inJobFunction, inNumDependencies)
			, name(inJobName)
		{}
		
		const char* name;
	};

	int GetMaxConcurrency() const override { return jobs::getWorkersCount(); }

	JPH::JobHandle CreateJob(const char *inName, JPH::ColorArg inColor, const JobFunction &inJobFunction, JPH::uint32 inNumDependencies = 0) override { 
		Job* job = new MyJob(inName, inColor, this, inJobFunction, inNumDependencies);
		if (inNumDependencies == 0) QueueJob(job);
		return JPH::JobHandle(job);
	}

	void FreeJob(Job *inJob) override {
		delete inJob;
	}

	void QueueJob(Job *inJob) override {
		jobs::run(inJob, [](void* data){
			MyJob* j = (MyJob*)data;
			PROFILE_BLOCK(j->name);
			j->Execute();
		}, nullptr);
	}

	void QueueJobs(Job **inJobs, JPH::uint inNumJobs) override {
		for (u32 i = 0; i < inNumJobs; ++i) QueueJob(inJobs[i]);
	}
};

struct JoltModuleImpl : JoltModule {
	struct MeshShape {
		JPH::Ref<JPH::MeshShape> shape;
		Model* model = nullptr;
	};

	struct Body {
		JPH::EMotionType motion_type = JPH::EMotionType::Static;
		JPH::ObjectLayer layer = Layers::NON_MOVING;
		JPH::Body* body = nullptr;
		JPH::Ref<JPH::Shape> shape;
		JPH::EMotionQuality motion_quality = JPH::EMotionQuality::LinearCast;
		float friction = 0.2f;
		float restitution = 0;
		float linear_damping = 0.05f;
		float angular_damping = 0.05f;
	};

	JoltModuleImpl(Engine& engine, ISystem& system, World& world, IAllocator& allocator)
		: m_engine(engine)
		, m_system(system)
		, m_world(world)
		, m_allocator(allocator)
		, m_bodies(m_allocator)
		, m_boxes(m_allocator)
		, m_meshes(m_allocator)
		, m_spheres(m_allocator)
		, m_callbacks_ref_count(m_allocator)
		, m_temp_allocator(10 * 1024 * 1024)
	{
		const u32 MAX_BODIES = 1024;
		const u32 MAX_BODY_PAIRS = 1024;
		const u32 MAX_CONTACT_CONSTRAINTS = 1024;
		m_jolt_system.Init(MAX_BODIES, 0, MAX_BODY_PAIRS, MAX_CONTACT_CONSTRAINTS, m_broad_phase_layer_interface, m_object_vs_broadphase_layer_filter, m_object_vs_object_layer_filter);
	}

	~JoltModuleImpl() {
		for (auto iter = m_callbacks_ref_count.begin(); iter.isValid(); ++iter) {
			iter.key()->getObserverCb().unbind<&JoltModuleImpl::onModelStateChanged>(this);
		}
		for (Body& body : m_bodies) {
			if (!body.body) continue;
			m_jolt_system.GetBodyInterface().DestroyBody(body.body->GetID());
		}
	}

	const char* getName() const override { return "jolt"; }

	bool isBodyActive(EntityRef entity) override {
		Body& body = m_bodies[entity];
		if (!body.body) return false;

		return body.body->IsActive();
	}

	float getFriction(EntityRef entity) override {
		const Body& body = m_bodies[entity];
		return body.friction;
	}

	float getRestitution(EntityRef entity) override {
		const Body& body = m_bodies[entity];
		return body.restitution;
	}

	float getLinearDamping(EntityRef entity) override {
		const Body& body = m_bodies[entity];
		return body.linear_damping;
	}

	float getAngularDamping(EntityRef entity) override {
		const Body& body = m_bodies[entity];
		return body.angular_damping;
	}

	void setLinearDamping(EntityRef entity, float value) override {
		Body& body = m_bodies[entity];
		body.linear_damping = value;
		if (body.body) {
			JPH::MotionProperties* motion_properties = body.body->GetMotionProperties();
			if (motion_properties) {
				motion_properties->SetLinearDamping(body.linear_damping);
			}
		}
	}

	struct StreamOut : JPH::StreamOut {
		StreamOut(IAllocator& allocator) : blob(allocator) {}
		void WriteBytes(const void* data, size_t num_bytes) override {
			failed = !blob.write(data, num_bytes) || failed; 
		}
		bool IsFailed() const override { return failed; }

		OutputMemoryStream blob;
		bool failed = false;
	};

	void takeSnapshot() {
		JPH::Ref<JPH::PhysicsScene> scene = new JPH::PhysicsScene();
		scene->FromPhysicsSystem(&m_jolt_system);

		StreamOut stream(m_allocator);
		scene->SaveBinaryState(stream, true, true);

		os::OutputFile file;
		if (file.open("jolt_snapshot.bin")) {
			if (!file.write(stream.blob.data(), stream.blob.size())) {
				logError("Failed to write jolt_snapshot.bin");
			}
			file.close();
		}
		else logError("Failed to write jolt_snapshot.bin");

	}

	void setAngularDamping(EntityRef entity, float value) override {
		Body& body = m_bodies[entity];
		body.angular_damping = value;
		if (body.body) {
			JPH::MotionProperties* motion_properties = body.body->GetMotionProperties();
			if (motion_properties) {
				motion_properties->SetAngularDamping(body.linear_damping);
			}
		}
	}

	void setFriction(EntityRef entity, float friction) override {
		Body& body = m_bodies[entity];
		body.friction = friction;
		if (body.body) body.body->SetFriction(friction);
	}

	void setRestitution(EntityRef entity, float restitution) override {
		Body& body = m_bodies[entity];
		body.restitution = restitution;
		if (body.body) body.body->SetRestitution(restitution);
	}

	bool isDiscreteMotion(EntityRef entity) override {
		Body& body = m_bodies[entity];
		return body.motion_quality == JPH::EMotionQuality::Discrete;
	}
	
	void setDiscreteMotion(EntityRef entity, bool is_descrete) override {
		Body& body = m_bodies[entity];
		body.motion_quality = is_descrete ? JPH::EMotionQuality::Discrete : JPH::EMotionQuality::LinearCast;
	}

	float getBodySpeed(EntityRef entity) override {
		Body& body = m_bodies[entity];
		if (!body.body) return false;

		return body.body->GetLinearVelocity().Length();
	}

	Vec3 getLinearVelocity(EntityRef entity) override {
		Body& body = m_bodies[entity];
		if (!body.body) return {};

		return toLumix(body.body->GetLinearVelocity());
	}

	void setLinearVelocity(EntityRef entity, const Vec3& velocity) override {
		Body& body = m_bodies[entity];
		if (!body.body) return;

		body.body->SetLinearVelocity(toJPH(velocity));
	}

	void addImpulse(EntityRef entity, const Vec3& impulse) {
		Body& body = m_bodies[entity];
		if (!body.body) return;

		JPH::BodyInterface& bi = m_jolt_system.GetBodyInterface();
		bi.AddImpulse(body.body->GetID(), toJPH(impulse));
	}

	void addForce(EntityRef entity, const Vec3& force) override {
		Body& body = m_bodies[entity];
		if (!body.body) return;

		body.body->AddForce(toJPH(force));
	}

	void serialize(struct OutputMemoryStream& serializer) override {
		serializer.write(m_bodies.size());
		for (auto iter = m_bodies.begin(); iter.isValid(); ++iter) {
			serializer.write(iter.key());
			const Body& body = iter.value();
			serializer.write(body.motion_type);
			serializer.write(body.layer);
			serializer.write(body.motion_quality);
			serializer.write(body.friction);
			serializer.write(body.restitution);
			serializer.write(body.linear_damping);
			serializer.write(body.angular_damping);
		}

		serializer.write(m_boxes.size());
		for (auto iter = m_boxes.begin(); iter.isValid(); ++iter) {
			serializer.write(iter.key());
			const JPH::Ref<JPH::BoxShape>& box = iter.value();
			const Vec3 half_extents = toLumix(box->GetHalfExtent());
			serializer.write(half_extents);
		}

		serializer.write(m_spheres.size());
		for (auto iter = m_spheres.begin(); iter.isValid(); ++iter) {
			serializer.write(iter.key());
			const float radius = iter.value()->GetRadius();
			serializer.write(radius);
		}

		serializer.write(m_meshes.size());
		for (auto iter = m_meshes.begin(); iter.isValid(); ++iter) {
			serializer.write(iter.key());
			const MeshShape& m = iter.value();
			serializer.writeString(m.model ? m.model->getPath().c_str() : "");
		}
	}

	i32 getVersion() const override { return (i32)JoltVersion::LATEST; }

	void deserialize(struct InputMemoryStream& serializer, const struct EntityMap& entity_map, i32 version) override {
		if (version <= (i32)JoltVersion::MESHES) {
			const u32 num = serializer.read<u32>();
			ASSERT(num == 0);
			return;
		}
		u32 num_bodies;
		serializer.read(num_bodies);
		Array<EntityRef> bodies(m_allocator); // TODO temporary allocation
		if (m_is_game_running) bodies.reserve(num_bodies);
		for (u32 i = 0; i < num_bodies; ++i) {
			EntityRef e;
			serializer.read(e);
			const EntityRef mapped = entity_map.get(e);
			createBody(mapped);
			Body& body = m_bodies[mapped];

			serializer.read(body.motion_type);
			serializer.read(body.layer);

			if (version > (i32)JoltVersion::BODY_PROPERTIES) {
				serializer.read(body.motion_quality);
				serializer.read(body.friction);
				serializer.read(body.restitution);
				serializer.read(body.linear_damping);
				serializer.read(body.angular_damping);
			}

			if (m_is_game_running) bodies.push(mapped);
		}

		u32 num_boxes;
		serializer.read(num_boxes);
		for (u32 i = 0; i < num_boxes; ++i) {
			EntityRef e;
			serializer.read(e);
			Vec3 he;
			serializer.read(he);

			const EntityRef mapped = entity_map.get(e);
			createBox(mapped);
			m_boxes[mapped] = new JPH::BoxShape(toJPH(he));
		}

		u32 num_spheres;
		serializer.read(num_spheres);
		for (u32 i = 0; i < num_spheres; ++i) {
			EntityRef e;
			serializer.read(e);
			float radius;
			serializer.read(radius);

			const EntityRef mapped = entity_map.get(e);
			createSphere(mapped);
			m_spheres[mapped] = new JPH::SphereShape(radius);
		}

		u32 mesh_count;
		serializer.read(mesh_count);
		ResourceManagerHub& rm = m_engine.getResourceManager();
		for (u32 i = 0; i < mesh_count; ++i) {
			EntityRef e;
			serializer.read(e);

			Path path(serializer.readString());

			const EntityRef mapped = entity_map.get(e);
			auto iter = m_meshes.insert(mapped, {});
			setMeshPath(mapped, path);
			m_world.onComponentCreated(mapped, JOLT_MESH_TYPE, this);
		}

		if (m_is_game_running) {
			for (EntityRef e : bodies) {
				createJoltBody(m_bodies[e], e);
			}
		}
	}

	ISystem& getSystem() const override { return m_system; }
	World& getWorld() override { return m_world; }

	void createJoltBody(Body& body, EntityRef entity) {
		JPH::MutableCompoundShapeSettings css;

		auto box_iter = m_boxes.find(entity);
		auto sphere_iter = m_spheres.find(entity);
		auto mesh_iter = m_meshes.find(entity);
		if (box_iter.isValid()) css.AddShape({0, 0, 0}, {0, 0, 0, 1}, box_iter.value());
		if (sphere_iter.isValid()) css.AddShape({0, 0, 0}, {0, 0, 0, 1}, sphere_iter.value());
		if (mesh_iter.isValid() && mesh_iter.value().shape.GetPtr()) css.AddShape({0, 0, 0}, {0, 0, 0, 1}, mesh_iter.value().shape); 
		for (EntityPtr c = m_world.getFirstChild(entity); c.isValid(); c = m_world.getNextSibling((EntityRef)c)) {
			EntityRef child = *c;
			box_iter = m_boxes.find(child);
			sphere_iter = m_spheres.find(child);
			mesh_iter = m_meshes.find(child);
			if (box_iter.isValid()) {
				const Transform local_tr = m_world.getLocalTransform(child);
				css.AddShape(toJPH(Vec3(local_tr.pos)), toJPH(local_tr.rot), box_iter.value());
			}
			if (sphere_iter.isValid()) {
				const Transform local_tr = m_world.getLocalTransform(child);
				css.AddShape(toJPH(Vec3(local_tr.pos)), toJPH(local_tr.rot), sphere_iter.value());
			}
			if (mesh_iter.isValid() && mesh_iter.value().model && mesh_iter.value().model->isReady()) {
				MeshShape& shape = mesh_iter.value();
				if (!shape.shape) createJoltMesh(shape);
				const Transform local_tr = m_world.getLocalTransform(child);
				css.AddShape(toJPH(Vec3(local_tr.pos)), toJPH(local_tr.rot), mesh_iter.value().shape);
			}
		}

		JPH::ShapeSettings::ShapeResult result = css.Create();
		if (!result.IsValid()) {
			ASSERT(false);
			return;
		}

		body.shape = result.Get();

		const DVec3 pos = m_world.getPosition(entity);
		JPH::BodyCreationSettings bcs(
			body.shape,
			{(float)pos.x, (float)pos.y, (float)pos.z},
			JPH::Quat::sIdentity(),
			body.motion_type,
			body.layer
		);
		bcs.mEnhancedInternalEdgeRemoval = true;
		bcs.mMotionQuality = body.motion_quality;
		bcs.mFriction = body.friction;
		bcs.mRestitution = body.restitution;
		bcs.mLinearDamping = body.linear_damping;
		bcs.mAngularDamping = body.angular_damping;
		JPH::BodyInterface& bi = m_jolt_system.GetBodyInterface();
		body.body = bi.CreateBody(bcs);
		body.body->SetUserData(entity.index);
		bi.AddBody(body.body->GetID(), JPH::EActivation::Activate);
	}
	
	void startGame() {
		m_is_game_running = true;
		for (auto iter = m_bodies.begin(); iter.isValid(); ++iter) {
			EntityRef body_entity = iter.key();
			Body& body = iter.value();
			createJoltBody(body, body_entity);
		}
		m_debug_renderer.module = (RenderModule*)m_world.getModule("renderer");
	}

	void stopGame() { m_is_game_running = false; }

	void enableDebugDraw(bool enable) override {
		m_is_debug_draw_enable = enable;
	}

	void drawDebug() {
		if (!m_is_debug_draw_enable) return;
		
		PROFILE_FUNCTION();
		JPH::BodyManager::DrawSettings draw_settings;
		draw_settings.mDrawShape = true;
		draw_settings.mDrawGetSupportingFace = true;
		draw_settings.mDrawShapeWireframe = true;
		draw_settings.mDrawBoundingBox = true;
		draw_settings.mDrawCenterOfMassTransform = true;
		draw_settings.mDrawVelocity = true;
		
		m_jolt_system.DrawBodies(draw_settings, &m_debug_renderer);
	}

	void update(float time_delta) override {
		PROFILE_FUNCTION();
		if (!m_is_game_running) return;
		
		m_jolt_system.Update(time_delta, 1, &m_temp_allocator, &m_job_system);
		
		JPH::BodyIDVector active_bodies;
		m_jolt_system.GetActiveBodies(JPH::EBodyType::RigidBody, active_bodies);
		JPH::BodyInterface& body_interface = m_jolt_system.GetBodyInterface();
		for (JPH::BodyID i : active_bodies) {
			const JPH::RVec3 pos = body_interface.GetPosition(i);
			const JPH::Quat rot = body_interface.GetRotation(i);
			EntityRef e = {(i32)m_jolt_system.GetBodyInterface().GetUserData(i)};
			m_world.setPosition(e, DVec3(toLumix(pos)));
			m_world.setRotation(e, toLumix(rot));
		}

		drawDebug();
	}

	void destroyBody(EntityRef entity) {
		auto iter = m_bodies.find(entity);
		if (!iter.isValid()) return;
		
		Body& body = iter.value();
		if (body.body) {
			JPH::BodyInterface& body_interface = m_jolt_system.GetBodyInterface();
			body_interface.RemoveBody(body.body->GetID());
			body_interface.DestroyBody(body.body->GetID());
		}
		
		m_bodies.erase(entity);
		m_world.onComponentDestroyed(entity, JOLT_BODY_TYPE, this);
	}

	void createBody(EntityRef entity) {
		m_bodies.insert(entity, {});
		m_world.onComponentCreated(entity, JOLT_BODY_TYPE, this);
	}

	void setSphereRadius(EntityRef entity, float radius) override {
		JPH::Ref<JPH::SphereShape> new_sphere = new JPH::SphereShape(radius);
		m_spheres[entity] = new_sphere;
		EntityPtr body_entity = getBodyEntity(entity);
		if (body_entity) recreateJoltBody(*body_entity);
	}

	float getSphereRadius(EntityRef entity) const override {
		return m_spheres[entity]->GetRadius();
	}

	void setBoxHalfExtents(EntityRef entity, Vec3 half_extents) override {
		JPH::Ref<JPH::BoxShape> new_box = new JPH::BoxShape(toJPH(half_extents));
		m_boxes[entity] = new_box;
		EntityPtr body_entity = getBodyEntity(entity);
		if (body_entity) recreateJoltBody(*body_entity);
	}

	Path getMeshPath(EntityRef entity) {
		const MeshShape& shape = m_meshes[entity];
		if (shape.model) return shape.model->getPath();
		return {};
	}

	void createJoltMesh(JoltModuleImpl::MeshShape& shape) {
		if (!shape.model) return;
		if (!shape.model->isReady()) return;

		JPH::VertexList vertices;
		JPH::IndexedTriangleList triangles;
		u32 index_offset = 0;
		for (u32 mesh_idx = 0; mesh_idx < (u32)shape.model->getMeshCount(); ++mesh_idx) {
			const Mesh& mesh = shape.model->getMesh(mesh_idx);
			const u32 num_vertex = mesh.vertices.size();
			const u32 num_index = mesh.indices_count;
			if (num_vertex == 0 || num_index < 3) continue;
			
			vertices.reserve(vertices.size() + num_vertex);
			
			const Vec3* positions = (const Vec3*)mesh.vertices.begin();
			for (u32 i = 0; i < num_vertex; ++i) {
				JPH::Float3& v = vertices.emplace_back();
				v.x = positions[i].x;
				v.y = positions[i].y;
				v.z = positions[i].z;
			}
			
			triangles.reserve(triangles.size() + num_index / 3);

			if (mesh.areIndices16()) {
				const u16* indices = (const u16*)mesh.indices.data();
				for (u32 i = 0; i + 2 < num_index; i += 3) {
					triangles.push_back(JPH::IndexedTriangle(index_offset + indices[i], index_offset + indices[i + 1], index_offset + indices[i + 2]));
				}
			} else {
				const u32* indices = (const u32*)mesh.indices.data();
				for (u32 i = 0; i + 2 < num_index; i += 3) {
					triangles.push_back(JPH::IndexedTriangle(index_offset + indices[i], index_offset + indices[i + 1], index_offset + indices[i + 2]));
				}
			}

			index_offset += num_vertex;
		}

		JPH::Ref<JPH::MeshShapeSettings> settings = new JPH::MeshShapeSettings(vertices, triangles);
		JPH::ShapeSettings::ShapeResult result = settings->Create();
		if (!result.IsValid()) return;

		shape.shape = (JPH::MeshShape*)result.Get().GetPtr();
	}

	void onModelStateChanged(Resource::State old_state, Resource::State new_state, Resource& resource) {
		if (old_state != Resource::State::READY && new_state == Resource::State::READY) {
			for (MeshShape& shape : m_meshes) {
				if (shape.model != &resource) continue;

				createJoltMesh(shape);
			}
		}
	}

	void setMeshPath(EntityRef entity, const Path& path) {
		MeshShape& shape = m_meshes[entity];
		if (shape.model) {
			auto cb_iter = m_callbacks_ref_count.find(shape.model);
			--cb_iter.value();
			if (cb_iter.value() == 0) {
				shape.model->getObserverCb().unbind<&JoltModuleImpl::onModelStateChanged>(this);
				m_callbacks_ref_count.erase(cb_iter);
			}
			shape.model->decRefCount();
		}
		shape.shape = nullptr;
		shape.model = path.isEmpty() ? nullptr : m_engine.getResourceManager().load<Model>(path);

		if (shape.model) {
			auto cb_iter = m_callbacks_ref_count.find(shape.model);
			if (cb_iter.isValid()) ++cb_iter.value();
			else {
				m_callbacks_ref_count.insert(shape.model, 1);
				shape.model->getObserverCb().bind<&JoltModuleImpl::onModelStateChanged>(this);
			}

			if (shape.model->isReady()) {
				createJoltMesh(shape);
			}
		}

		if (!shape.model || shape.model->isReady()) {
			EntityPtr body_entity = getBodyEntity(entity);
			if (body_entity) recreateJoltBody(*body_entity);
		}
	}

	EntityPtr getBodyEntity(EntityRef shape_entity) {
		if (m_bodies.find(shape_entity).isValid()) return shape_entity;
		
		EntityPtr parent = m_world.getParent(shape_entity);
		if (!parent.isValid()) return INVALID_ENTITY;
		if (m_bodies.find(*parent).isValid()) return parent;
		return INVALID_ENTITY;
	}

	void recreateJoltBody(EntityRef entity) {
		if (!m_is_game_running) return;

		Body& body = m_bodies[entity];

		if (body.body) {
			JPH::BodyInterface& body_interface = m_jolt_system.GetBodyInterface();
			body_interface.RemoveBody(body.body->GetID());
			body_interface.DestroyBody(body.body->GetID());
		}
		
		createJoltBody(body, entity);
	}

	void destroyMesh(EntityRef entity) override {
		auto iter = m_meshes.find(entity);
		if (!iter.isValid()) return;

		EntityPtr body_entity = getBodyEntity(entity);
		if (body_entity.isValid()) recreateJoltBody(*body_entity);

		m_meshes.erase(entity);
		m_world.onComponentDestroyed(entity, JOLT_MESH_TYPE, this);
	}

	void destroyBox(EntityRef entity) override {
		auto iter = m_boxes.find(entity);
		if (!iter.isValid()) return;

		EntityPtr body_entity = getBodyEntity(entity);
		if (body_entity.isValid()) recreateJoltBody(*body_entity);

		m_boxes.erase(entity);
		m_world.onComponentDestroyed(entity, JOLT_BOX_TYPE, this);
	}

	void destroySphere(EntityRef entity) override {
		auto iter = m_spheres.find(entity);
		if (!iter.isValid()) return;

		EntityPtr body_entity = getBodyEntity(entity);
		if (body_entity.isValid()) recreateJoltBody(*body_entity);

		m_spheres.erase(entity);
		m_world.onComponentDestroyed(entity, JOLT_SPHERE_TYPE, this);
	}

	void createSphere(EntityRef entity) override {
		m_spheres.insert(entity, new JPH::SphereShape(1));
		m_world.onComponentCreated(entity, JOLT_SPHERE_TYPE, this);
	}

	void createMesh(EntityRef entity) override {
		m_meshes.insert(entity, {});
		m_world.onComponentCreated(entity, JOLT_MESH_TYPE, this);
	}

	void createBox(EntityRef entity) override {
		m_boxes.insert(entity, new JPH::BoxShape({1, 1, 1}));
		m_world.onComponentCreated(entity, JOLT_BOX_TYPE, this);
	}
	
	Vec3 getBoxHalfExtents(EntityRef entity) const override {
		return toLumix(m_boxes[entity]->GetHalfExtent());
	}

	ObjectLayer getLayer(EntityRef entity) override {
		const Body& body = m_bodies[entity];
		return (ObjectLayer)body.layer;
	}

	void setLayer(EntityRef entity, ObjectLayer new_layer) override {
		ASSERT((u32)new_layer <= 0xffFF);
		Body& body = m_bodies[entity];
		body.layer = (JPH::ObjectLayer)new_layer;
		
		if (!body.body) return;
		if (body.body->GetObjectLayer() == new_layer) return;
		
		JPH::BodyInterface& body_interface = m_jolt_system.GetBodyInterface();
		
		const DVec3 pos = m_world.getPosition(entity);
		JPH::EMotionType motion_type = body.body->GetMotionType();
		JPH::Ref<JPH::Shape> shape = body.shape;
		
		body_interface.RemoveBody(body.body->GetID());
		body_interface.DestroyBody(body.body->GetID());
		
		JPH::BodyCreationSettings bcs(shape, {(float)pos.x, (float)pos.y, (float)pos.z}, JPH::Quat::sIdentity(), motion_type, new_layer);
		bcs.mEnhancedInternalEdgeRemoval = true;
		bcs.mMotionQuality = body.motion_quality;
		bcs.mFriction = body.friction;
		bcs.mRestitution = body.restitution;
		bcs.mLinearDamping = body.linear_damping;
		bcs.mAngularDamping = body.angular_damping;

		body.body = body_interface.CreateBody(bcs);
		body.body->SetUserData(entity.index);
		body_interface.AddBody(body.body->GetID(), JPH::EActivation::Activate);
	}

	void setDynamicType(EntityRef entity, JPH::EMotionType type) override {
		Body& body = m_bodies[entity];
		body.motion_type = type;
		if (body.body) body.body->SetMotionType(type);
	}

	JPH::EMotionType getDynamicType(EntityRef entity) override {
		const Body& body = m_bodies[entity];
		return body.motion_type;
	}

	static void reflect() {
		struct DynamicTypeEnum : reflection::EnumAttribute {
			u32 count(ComponentUID cmp) const override { return 3; }
			const char* name(ComponentUID cmp, u32 idx) const override { 
				switch ((JPH::EMotionType)idx) {
					case JPH::EMotionType::Dynamic: return "Dynamic";
					case JPH::EMotionType::Static: return "Static";
					case JPH::EMotionType::Kinematic: return "Kinematic";
				}
				ASSERT(false);
				return "N/A";
			}
		};

		struct LayerEnum : reflection::EnumAttribute {
			u32 count(ComponentUID cmp) const override { return 3; }
			const char* name(ComponentUID cmp, u32 idx) const override { 
				switch ((JPH::ObjectLayer)idx) {
					case Layers::MOVING: return "Moving";
					case Layers::NON_MOVING: return "Non-moving";
					case Layers::NUM_LAYERS: return "Error";
				}
				ASSERT(false);
				return "N/A";
			}
		};

		#include "jolt_module.gen.h"
	}

	IAllocator& m_allocator;
	Engine& m_engine;
	ISystem& m_system;
	World& m_world;

	JPH::TempAllocatorImpl m_temp_allocator; // TODO lumix temp allocator?
	JoltJobSystemProxy m_job_system;
	BPLayerInterfaceImpl m_broad_phase_layer_interface;
	ObjectVsBroadPhaseLayerFilterImpl m_object_vs_broadphase_layer_filter;
	ObjectLayerPairFilterImpl m_object_vs_object_layer_filter;
	JPH::PhysicsSystem m_jolt_system;
	bool m_is_game_running = false;
	bool m_is_debug_draw_enable = false;

	HashMap<EntityRef, Body> m_bodies;
	HashMap<EntityRef, JPH::Ref<JPH::BoxShape>> m_boxes;
	HashMap<EntityRef, MeshShape> m_meshes;
	HashMap<EntityRef, JPH::Ref<JPH::SphereShape>> m_spheres;
	HashMap<Model*, u32> m_callbacks_ref_count;
	JoltDebugRenderer m_debug_renderer;
};


struct JoltSystem : ISystem {
	JoltSystem(Engine& engine)
		: m_engine(engine)
		, m_allocator(engine.getAllocator(), "jolt")
	{
		s_allocator = &m_allocator;
		JPH::AlignedAllocate = &JoltSystem::jph_alignedAllocate;
		JPH::AlignedFree = &JoltSystem::jph_alignedFree;
		JPH::Allocate = &JoltSystem::jph_allocate;
		JPH::Reallocate = &JoltSystem::jph_reallocate;
		JPH::Free = &JoltSystem::jph_free;

		JPH::Factory::sInstance = new JPH::Factory();
		JPH::RegisterTypes();
		JoltModuleImpl::reflect();
	}

	~JoltSystem() {
		JPH::UnregisterTypes();
		delete JPH::Factory::sInstance;
		JPH::Factory::sInstance = nullptr;

		JPH::AlignedAllocate = nullptr;
		JPH::AlignedFree = nullptr;
		JPH::Allocate = nullptr;
		JPH::Reallocate = nullptr;
		JPH::Free = nullptr;

		s_allocator = nullptr;
	}

	static void* jph_reallocate(void *mem, size_t old_size, size_t new_size) {
		return s_allocator->reallocate(mem, new_size, old_size, 8);
	}

	static void jph_alignedFree(void* mem) {
		s_allocator->deallocate(mem);
	}
	
	static void jph_free(void* mem) {
		s_allocator->deallocate(mem);
	}

	static void* jph_alignedAllocate(size_t size, size_t align) {
		return s_allocator->allocate(size, align);
	}

	static void* jph_allocate(size_t size) {
		return s_allocator->allocate(size, 8);
	}

	const char* getName() const override { return "jolt"; }
	
	void serialize(OutputMemoryStream& serializer) const override {}
	bool deserialize(i32 version, InputMemoryStream& serializer) override {
		return version == 0;
	}

	void createModules(World& world) override {
		IAllocator& allocator = m_engine.getAllocator();
		UniquePtr<JoltModuleImpl> module = UniquePtr<JoltModuleImpl>::create(allocator, m_engine, *this, world, allocator);
		world.addModule(module.move());
	}

	Engine& m_engine;
	TagAllocator m_allocator;
	static IAllocator* s_allocator;
};

IAllocator* JoltSystem::s_allocator = nullptr;

LUMIX_PLUGIN_ENTRY(jolt)
{
	return LUMIX_NEW(engine.getAllocator(), JoltSystem)(engine);
}

}