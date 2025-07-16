#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include "core/hash_map.h"
#include "core/job_system.h"
#include "core/profiler.h"
#include "core/stream.h"
#include "core/string.h"
#include "engine/engine.h"
#include "engine/plugin.h"
#include "engine/reflection.h"
#include "engine/world.h"
#include "imgui/IconsFontAwesome5.h"

namespace Lumix {

static const ComponentType JOLT_BODY_TYPE = reflection::getComponentType("jolt_body");

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

struct JoltModule : IModule {
	JoltModule(Engine& engine, ISystem& system, World& world, IAllocator& allocator)
		: m_engine(engine)
		, m_system(system)
		, m_world(world)
		, m_allocator(allocator)
		, m_bodies(m_allocator)
		, m_temp_allocator(10 * 1024 * 1024)
	{
		const u32 MAX_BODIES = 1024;
		const u32 MAX_BODY_PAIRS = 1024;
		const u32 MAX_CONTACT_CONSTRAINTS = 1024;
		m_jolt_system.Init(MAX_BODIES, 0, MAX_BODY_PAIRS, MAX_CONTACT_CONSTRAINTS, m_broad_phase_layer_interface, m_object_vs_broadphase_layer_filter, m_object_vs_object_layer_filter);
	}

	~JoltModule() {
		for (Body& body : m_bodies) {
			if (!body.body) continue;
			m_jolt_system.GetBodyInterface().DestroyBody(body.body->GetID());
		}
	}

	const char* getName() const override { return "jolt"; }

	void serialize(struct OutputMemoryStream& serializer) override {
		serializer.write(m_bodies.size());
		for (auto iter = m_bodies.begin(); iter.isValid(); ++iter) {
			serializer.write(iter.key());
			JPH::Body* body = iter.value().body;
			if (!body) {
				serializer.write(JPH::EShapeType::Empty);
				continue;
			}
			auto* compound = (JPH::CompoundShape*)body->GetShape();
			serializer.write(compound->GetType());
			serializer.write(body->GetMotionType());
			serializer.write(body->GetObjectLayer());
			serializer.write(compound->GetSubType());

			const u32 num_sub_shapes = compound->GetNumSubShapes();
			serializer.write(num_sub_shapes);
					
			for (JPH::uint j = 0; j < num_sub_shapes; ++j) {
				const JPH::CompoundShape::SubShape& sub_shape = compound->GetSubShape(j);
				const JPH::Shape* sub_shape_ptr = sub_shape.mShape;
				
				serializer.write(sub_shape_ptr->GetType());
				serializer.write(sub_shape_ptr->GetSubType());
				
				const JPH::Vec3 pos = sub_shape.GetPositionCOM();
				const Vec3 position = {pos.GetX(), pos.GetY(), pos.GetZ()};
				serializer.write(position);
				
				const JPH::Quat rot = sub_shape.GetRotation();
				const Quat rotation = {rot.GetX(), rot.GetY(), rot.GetZ(), rot.GetW()};
				serializer.write(rotation);
				
				switch (sub_shape_ptr->GetType()) {
					case JPH::EShapeType::Convex:
						switch (sub_shape_ptr->GetSubType()) {
							case JPH::EShapeSubType::Box: {
								auto* box = (JPH::BoxShape*)sub_shape_ptr;
								const JPH::Vec3 size = box->GetHalfExtent();
								const Vec3 v3 = {size.GetX(), size.GetY(), size.GetZ()};
								serializer.write(v3);
								break;
							}
							default: ASSERT(false); break;
						}
						break;
					default: ASSERT(false); break;
				}
			}
		}
	}

	void deserialize(struct InputMemoryStream& serializer, const struct EntityMap& entity_map, i32 version) override {
		const u32 num = serializer.read<u32>();
		JPH::BodyInterface& body_interface = m_jolt_system.GetBodyInterface();
		for (u32 i = 0; i < num; ++i) {
			EntityRef entity = serializer.read<EntityRef>();
			entity = entity_map.get(entity);
			Body& body = m_bodies.insert(entity);
			m_world.onComponentCreated(entity, JOLT_BODY_TYPE, this);

			const JPH::EShapeType type = serializer.read<JPH::EShapeType>();
			if (type == JPH::EShapeType::Empty) continue;

			const JPH::EMotionType motion_type = serializer.read<JPH::EMotionType>();
			const JPH::ObjectLayer layer = serializer.read<JPH::ObjectLayer>();
			const JPH::EShapeSubType subtype = serializer.read<JPH::EShapeSubType>();
			const DVec3 pos = m_world.getPosition(entity);

			ASSERT(type == JPH::EShapeType::Compound);
			const u32 num_sub_shapes = serializer.read<u32>();
			JPH::MutableCompoundShapeSettings css;
			
			for (JPH::uint j = 0; j < num_sub_shapes; ++j) {
				const JPH::EShapeType sub_shape_type = serializer.read<JPH::EShapeType>();
				const JPH::EShapeSubType sub_shape_subtype = serializer.read<JPH::EShapeSubType>();
				
				const Vec3 position = serializer.read<Vec3>();
				const Quat rotation = serializer.read<Quat>();
				
				JPH::Vec3 jph_pos(position.x, position.y, position.z);
				JPH::Quat jph_rot(rotation.x, rotation.y, rotation.z, rotation.w);
				
				JPH::Ref<JPH::Shape> sub_shape;
				switch (sub_shape_type) {
					case JPH::EShapeType::Convex:
						switch (sub_shape_subtype) {
							case JPH::EShapeSubType::Box: {
								const Vec3 size = serializer.read<Vec3>();
								sub_shape = new JPH::BoxShape({size.x, size.y, size.z});
								break;
							}
							default: ASSERT(false); break;
						}
						break;
					default: ASSERT(false); break;
				}
				
				css.AddShape(jph_pos, jph_rot, sub_shape);
			}
			
			JPH::ShapeSettings::ShapeResult result = css.Create();
			ASSERT(result.IsValid());
			body.shape = result.Get();
			
			JPH::BodyCreationSettings bcs(body.shape, {(float)pos.x, (float)pos.y, (float)pos.z}, JPH::Quat::sIdentity(), motion_type, layer);
			body.body = m_jolt_system.GetBodyInterface().CreateBody(bcs);
			body.body->SetUserData(entity.index);
			body_interface.AddBody(body.body->GetID(), JPH::EActivation::Activate);
		}
	}

	ISystem& getSystem() const override { return m_system; }
	World& getWorld() override { return m_world; }
	
	void startGame() { m_is_game_running = true; }
	void stopGame() { m_is_game_running = false; }

	void update(float time_delta) override {
		PROFILE_FUNCTION();
		if (!m_is_game_running) return;
		
		m_jolt_system.Update(time_delta, 1, &m_temp_allocator, &m_job_system);
		
		JPH::BodyIDVector active_bodies;
		m_jolt_system.GetActiveBodies(JPH::EBodyType::RigidBody, active_bodies);
		for (JPH::BodyID i : active_bodies) {
			const JPH::RVec3 pos = m_jolt_system.GetBodyInterface().GetPosition(i);
			EntityRef e = {(i32)m_jolt_system.GetBodyInterface().GetUserData(i)};
			m_world.setPosition(e, {pos.GetX(), pos.GetY(), pos.GetZ()});
		}
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

	void addBoxGeometry(EntityRef entity, u32 idx) {
		Body& body = m_bodies[entity];
		const DVec3 pos = m_world.getPosition(entity);
		
		JPH::EMotionType motion_type = JPH::EMotionType::Dynamic;
		JPH::ObjectLayer layer = Layers::NON_MOVING;
		
		if (body.body) {
			motion_type = body.body->GetMotionType();
			layer = body.body->GetObjectLayer();
			
			JPH::BodyInterface& body_interface = m_jolt_system.GetBodyInterface();
			body_interface.RemoveBody(body.body->GetID());
			body_interface.DestroyBody(body.body->GetID());
		}

		JPH::MutableCompoundShapeSettings css;
		JPH::Ref<JPH::Shape> box_shape = new JPH::BoxShape({1, 1, 1});
		css.AddShape({0, 0, 0}, JPH::Quat::sIdentity(), box_shape);

		if (body.shape && body.shape->GetType() == JPH::EShapeType::Compound) {
			const JPH::CompoundShape* compound = static_cast<const JPH::CompoundShape*>(body.shape.GetPtr());
			for (JPH::uint i = 0; i < compound->GetNumSubShapes(); ++i) {
				const JPH::CompoundShape::SubShape& sub_shape = compound->GetSubShape(i);
				css.AddShape(sub_shape.GetPositionCOM(), sub_shape.GetRotation(), sub_shape.mShape);
			}
		}
		else if (body.shape) {
			css.AddShape({0, 0, 0}, JPH::Quat::sIdentity(), body.shape);
		}

		JPH::ShapeSettings::ShapeResult result = css.Create();
		ASSERT(result.IsValid());
		body.shape = result.Get();

		JPH::BodyCreationSettings bcs(body.shape, {(float)pos.x, (float)pos.y, (float)pos.z}, JPH::Quat::sIdentity(), motion_type, layer);
		JPH::BodyInterface& body_interface = m_jolt_system.GetBodyInterface();
		body.body = body_interface.CreateBody(bcs);
		body.body->SetUserData(entity.index);
		body_interface.AddBody(body.body->GetID(), JPH::EActivation::Activate);
	}

	void removeBoxGeometry(EntityRef entity, u32 idx) {
		Body& body = m_bodies[entity];
		if (!body.body || !body.shape) return;
		
		const DVec3 pos = m_world.getPosition(entity);
		
		JPH::EMotionType motion_type = body.body->GetMotionType();
		JPH::ObjectLayer layer = body.body->GetObjectLayer();
		
		JPH::BodyInterface& body_interface = m_jolt_system.GetBodyInterface();
		body_interface.RemoveBody(body.body->GetID());
		body_interface.DestroyBody(body.body->GetID());
		
		if (body.shape->GetType() == JPH::EShapeType::Compound) {
			const JPH::CompoundShape* compound = static_cast<const JPH::CompoundShape*>(body.shape.GetPtr());
			JPH::MutableCompoundShapeSettings css;
			
			u32 box_count = 0;
			for (JPH::uint i = 0; i < compound->GetNumSubShapes(); ++i) {
				const JPH::CompoundShape::SubShape& sub_shape = compound->GetSubShape(i);
				if (sub_shape.mShape->GetSubType() == JPH::EShapeSubType::Box) {
					if (box_count != idx) {
						css.AddShape(sub_shape.GetPositionCOM(), sub_shape.GetRotation(), sub_shape.mShape);
					}
					box_count++;
				} else {
					css.AddShape(sub_shape.GetPositionCOM(), sub_shape.GetRotation(), sub_shape.mShape);
				}
			}
			
			if (css.mSubShapes.size() > 0) {
				JPH::ShapeSettings::ShapeResult result = css.Create();
				ASSERT(result.IsValid());
				body.shape = result.Get();
				
				JPH::BodyCreationSettings bcs(body.shape, {(float)pos.x, (float)pos.y, (float)pos.z}, JPH::Quat::sIdentity(), motion_type, layer);
				body.body = body_interface.CreateBody(bcs);
				body.body->SetUserData(entity.index);
				body_interface.AddBody(body.body->GetID(), JPH::EActivation::Activate);
			} else {
				body.shape = nullptr;
				body.body = nullptr;
			}
		} else if (body.shape->GetSubType() == JPH::EShapeSubType::Box && idx == 0) {
			body.shape = nullptr;
			body.body = nullptr;
		}
	}

	void setBoxGeomHalfExtents(EntityRef entity, u32 idx, Vec3 half_extents) {
		Body& body = m_bodies[entity];
		if (!body.body || !body.shape) return;
		
		const DVec3 pos = m_world.getPosition(entity);
		JPH::EMotionType motion_type = body.body->GetMotionType();
		JPH::ObjectLayer layer = body.body->GetObjectLayer();
		
		JPH::BodyInterface& body_interface = m_jolt_system.GetBodyInterface();
		body_interface.RemoveBody(body.body->GetID());
		body_interface.DestroyBody(body.body->GetID());
		
		if (body.shape->GetSubType() == JPH::EShapeSubType::Box && idx == 0) {
			body.shape = new JPH::BoxShape({half_extents.x, half_extents.y, half_extents.z});
		}
		else if (body.shape->GetType() == JPH::EShapeType::Compound) {
			const JPH::CompoundShape* compound = static_cast<const JPH::CompoundShape*>(body.shape.GetPtr());
			JPH::MutableCompoundShapeSettings css;
			
			u32 box_count = 0;
			for (JPH::uint i = 0; i < compound->GetNumSubShapes(); ++i) {
				const JPH::CompoundShape::SubShape& sub_shape = compound->GetSubShape(i);
				if (sub_shape.mShape->GetSubType() == JPH::EShapeSubType::Box) {
					if (box_count == idx) {
						JPH::Ref<JPH::Shape> new_box = new JPH::BoxShape({half_extents.x, half_extents.y, half_extents.z});
						css.AddShape(sub_shape.GetPositionCOM(), sub_shape.GetRotation(), new_box);
					} else {
						css.AddShape(sub_shape.GetPositionCOM(), sub_shape.GetRotation(), sub_shape.mShape);
					}
					box_count++;
				} else {
					css.AddShape(sub_shape.GetPositionCOM(), sub_shape.GetRotation(), sub_shape.mShape);
				}
			}
			
			JPH::ShapeSettings::ShapeResult result = css.Create();
			ASSERT(result.IsValid());
			body.shape = result.Get();
		}
		
		JPH::BodyCreationSettings bcs(body.shape, {(float)pos.x, (float)pos.y, (float)pos.z}, JPH::Quat::sIdentity(), motion_type, layer);
		body.body = body_interface.CreateBody(bcs);
		body.body->SetUserData(entity.index);
		body_interface.AddBody(body.body->GetID(), JPH::EActivation::Activate);
	}

	const JPH::Shape* getShape(JPH::Body& body, JPH::EShapeSubType type, u32 idx) const {
		const JPH::Shape* shape = body.GetShape();
		if (shape->GetSubType() == type) return shape;
		
		if (shape->GetType() == JPH::EShapeType::Compound) {
			const JPH::CompoundShape* compound = static_cast<const JPH::CompoundShape*>(shape);
			u32 found_count = 0;
			for (JPH::uint i = 0; i < compound->GetNumSubShapes(); ++i) {
				const JPH::CompoundShape::SubShape& sub_shape = compound->GetSubShape(i);
				if (sub_shape.mShape->GetSubType() == type) {
					if (found_count == idx) {
						return sub_shape.mShape;
					}
					found_count++;
				}
			}
		}
		
		return nullptr;
	}

	Vec3 getBoxGeomHalfExtents(EntityRef entity, u32 idx) const {
		const Body& body = m_bodies[entity];
		const JPH::Shape* shape = getShape(*body.body, JPH::EShapeSubType::Box, idx);
		auto* box = (JPH::BoxShape*)shape;
		JPH::Vec3 he = box->GetHalfExtent();
		return {he.GetX(), he.GetY(), he.GetZ()};
	}

	u32 getBoxGeometryCount(EntityRef entity) const {
		const Body& body = m_bodies[entity];
		if (!body.body) return 0;
		
		const JPH::Shape* shape = body.body->GetShape();
		if (!shape) return 0;
		
		if (shape->GetType() == JPH::EShapeType::Compound) {
			const JPH::CompoundShape* compound = static_cast<const JPH::CompoundShape*>(shape);
			u32 box_count = 0;
			for (JPH::uint i = 0; i < compound->GetNumSubShapes(); ++i) {
				const JPH::CompoundShape::SubShape& sub_shape = compound->GetSubShape(i);
				if (sub_shape.mShape->GetSubType() == JPH::EShapeSubType::Box) {
					box_count++;
				}
			}
			return box_count;
		} else if (shape->GetSubType() == JPH::EShapeSubType::Box) {
			return 1;
		}
		
		return 0;
	}

	JPH::ObjectLayer getLayer(EntityRef entity) {
		const Body& body = m_bodies[entity];
		if (!body.body) return Layers::NON_MOVING;

		return body.body->GetObjectLayer();
	}

	void setLayer(EntityRef entity, JPH::ObjectLayer new_layer) {
		Body& body = m_bodies[entity];
		if (!body.body) return;
		if (body.body->GetObjectLayer() == new_layer) return;
		
		JPH::BodyInterface& body_interface = m_jolt_system.GetBodyInterface();
		
		const DVec3 pos = m_world.getPosition(entity);
		JPH::EMotionType motion_type = body.body->GetMotionType();
		JPH::Ref<JPH::Shape> shape = body.shape;
		
		body_interface.RemoveBody(body.body->GetID());
		body_interface.DestroyBody(body.body->GetID());
		
		JPH::BodyCreationSettings bcs(shape, {(float)pos.x, (float)pos.y, (float)pos.z}, JPH::Quat::sIdentity(), motion_type, new_layer);
		body.body = body_interface.CreateBody(bcs);
		body.body->SetUserData(entity.index);
		body_interface.AddBody(body.body->GetID(), JPH::EActivation::Activate);
	}

	void setDynamicType(EntityRef entity, JPH::EMotionType type) {
		const Body& body = m_bodies[entity];
		if (!body.body) return;

		body.body->SetMotionType(type);
	}

	JPH::EMotionType getDynamicType(EntityRef entity) {
		const Body& body = m_bodies[entity];
		if (!body.body) return JPH::EMotionType::Static;

		return body.body->GetMotionType();
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

		LUMIX_MODULE(JoltModule, "jolt")
			.LUMIX_CMP(Body, "jolt_body", "Jolt / Body")
			.icon(ICON_FA_VOLLEYBALL_BALL)
			.LUMIX_ENUM_PROP(DynamicType, "Dynamic").attribute<DynamicTypeEnum>()
			.LUMIX_ENUM_PROP(Layer, "Layer").attribute<LayerEnum>()
			.begin_array<&JoltModule::getBoxGeometryCount, &JoltModule::addBoxGeometry, &JoltModule::removeBoxGeometry>("Box geometry")	
				.LUMIX_PROP(BoxGeomHalfExtents, "Half extents")
				//.LUMIX_PROP(BoxGeomOffsetPosition, "Position offset")
				//.LUMIX_PROP(BoxGeomOffsetRotation, "Rotation offset").radiansAttribute()
			.end_array()
			/*.LUMIX_FUNC_EX(JoltModule::putToSleep, "putToSleep")
			.LUMIX_FUNC_EX(JoltModule::getActorSpeed, "getSpeed")
			.LUMIX_FUNC_EX(JoltModule::getActorVelocity, "getVelocity")
			.LUMIX_FUNC_EX(JoltModule::applyForceToActor, "applyForce")
			.LUMIX_FUNC_EX(JoltModule::applyImpulseToActor, "applyImpulse")
			.LUMIX_FUNC_EX(JoltModule::addForceAtPos, "addForceAtPos")
			.LUMIX_PROP(IsTrigger, "Trigger")
			.begin_array<&JoltModule::getSphereGeometryCount, &JoltModule::addSphereGeometry, &JoltModule::removeSphereGeometry>("Sphere geometry")
				.LUMIX_PROP(SphereGeomRadius, "Radius").minAttribute(0)
				.LUMIX_PROP(SphereGeomOffsetPosition, "Position offset")
			.end_array()
			.LUMIX_PROP(RigidActorCCD, "CCD")
			.LUMIX_PROP(MeshGeomPath, "Mesh").resourceAttribute(PhysicsGeometry::TYPE)
			.LUMIX_PROP(RigidActorMaterial, "Material").resourceAttribute(PhysicsMaterial::TYPE)
			*/
			;

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

	struct Body {
		JPH::Body* body = nullptr;
		JPH::Ref<JPH::Shape> shape;
	};

	HashMap<EntityRef, Body> m_bodies;
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
		JoltModule::reflect();
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
		UniquePtr<JoltModule> module = UniquePtr<JoltModule>::create(allocator, m_engine, *this, world, allocator);
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