#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include "core/hash_map.h"
#include "core/stream.h"
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
			case Layers::NON_MOVING: return inObject2 == Layers::MOVING; // Non moving only collides with moving
			case Layers::MOVING: return true;							 // Moving collides with everything
			default: ASSERT(false); return false;
		}
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
		, m_job_system(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, 5)
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

	void serialize(struct OutputMemoryStream& serializer) override {}
	void deserialize(struct InputMemoryStream& serializer, const struct EntityMap& entity_map, i32 version) override {}
	ISystem& getSystem() const override { return m_system; }
	World& getWorld() override { return m_world; }
	
	void startGame() { m_is_game_running = true; }
	void stopGame() { m_is_game_running = false; }

	void update(float time_delta) override {
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
		// TODO
		ASSERT(false);
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
		if (!body.body) {
			JPH::BodyCreationSettings bcs(body.shape, {(float)pos.x, (float)pos.y, (float)pos.z}, JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::NON_MOVING);
			body.body = m_jolt_system.GetBodyInterface().CreateBody(bcs);
			body.body->SetUserData(entity.index);
			m_jolt_system.GetBodyInterface().AddBody(body.body->GetID(), JPH::EActivation::Activate);
		}
	}

	void removeBoxGeometry(EntityRef entity, u32 idx) {
		
	}

	u32 getBoxGeometryCount(EntityRef entity) const {
		const Body& body = m_bodies[entity];
		if (!body.body) return 0;
		return 1;
	}

	static void reflect() {
		LUMIX_MODULE(JoltModule, "jolt")
			.LUMIX_CMP(Body, "jolt_body", "Jolt / Body")
			.icon(ICON_FA_VOLLEYBALL_BALL)
			.begin_array<&JoltModule::getBoxGeometryCount, &JoltModule::addBoxGeometry, &JoltModule::removeBoxGeometry>("Box geometry")	
				//.LUMIX_PROP(BoxGeomHalfExtents, "Size")
				//.LUMIX_PROP(BoxGeomOffsetPosition, "Position offset")
				//.LUMIX_PROP(BoxGeomOffsetRotation, "Rotation offset").radiansAttribute()
			.end_array()
			/*.LUMIX_FUNC_EX(JoltModule::putToSleep, "putToSleep")
			.LUMIX_FUNC_EX(JoltModule::getActorSpeed, "getSpeed")
			.LUMIX_FUNC_EX(JoltModule::getActorVelocity, "getVelocity")
			.LUMIX_FUNC_EX(JoltModule::applyForceToActor, "applyForce")
			.LUMIX_FUNC_EX(JoltModule::applyImpulseToActor, "applyImpulse")
			.LUMIX_FUNC_EX(JoltModule::addForceAtPos, "addForceAtPos")
			.LUMIX_ENUM_PROP(ActorLayer, "Layer").attribute<LayerEnum>()
			.LUMIX_ENUM_PROP(DynamicType, "Dynamic").attribute<DynamicTypeEnum>()
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
	JPH::JobSystemThreadPool m_job_system; // TODO use lumix's job system
	BPLayerInterfaceImpl m_broad_phase_layer_interface;
	ObjectVsBroadPhaseLayerFilterImpl m_object_vs_broadphase_layer_filter;
	ObjectLayerPairFilterImpl m_object_vs_object_layer_filter;
	JPH::PhysicsSystem m_jolt_system;
	bool m_is_game_running = false;

	struct Body {
		Body()
		{
			shape = new JPH::BoxShape({1, 1, 1});
			shape->SetEmbedded();
		}

		JPH::Body* body = nullptr;
		JPH::Shape* shape = nullptr;
	};

	HashMap<EntityRef, Body> m_bodies;
};


struct JoltSystem : ISystem {
	JoltSystem(Engine& engine)
		: m_engine(engine)
	{
		JPH::RegisterDefaultAllocator();
		JPH::Factory::sInstance = new JPH::Factory();
		JPH::RegisterTypes();
		JoltModule::reflect();
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
};


LUMIX_PLUGIN_ENTRY(jolt)
{
	return LUMIX_NEW(engine.getAllocator(), JoltSystem)(engine);
}

}