#include "engine/plugin.h"

namespace JPH {
	//@ enum full JPH::EMotionType
	enum class EMotionType : Lumix::u8;
}

namespace Lumix {

//@ enum
enum ObjectLayer : u16 {};

//@ module JoltModule jolt "Jolt"
struct JoltModule : IModule {
	//@ functions
	virtual void enableDebugDraw(bool enable) = 0;
	virtual void takeSnapshot() = 0;
	//@ end

	virtual void createSphere(EntityRef entity) = 0;
	virtual void destroySphere(EntityRef entity) = 0;

	virtual void createBox(EntityRef entity) = 0;
	virtual void destroyBox(EntityRef entity) = 0;

	virtual void createMesh(EntityRef entity) = 0;
	virtual void destroyMesh(EntityRef entity) = 0;

	virtual void createBody(EntityRef entity) = 0;
	virtual void destroyBody(EntityRef entity) = 0;

	//@ component Mesh id jolt_mesh
	virtual void setMeshPath(EntityRef entity, const Path& path) = 0; //@ resource_type Model::TYPE
	virtual Path getMeshPath(EntityRef entity) = 0;
	//@ end

	//@ component Sphere icon ICON_FA_GLOBE id jolt_sphere
	virtual float getSphereRadius(EntityRef entity) const = 0;
	virtual void setSphereRadius(EntityRef entity, float radius) = 0;
	//@ end
	
	//@ component Box icon ICON_FA_BOX id jolt_box
	virtual Vec3 getBoxHalfExtents(EntityRef entity) const = 0;
	virtual void setBoxHalfExtents(EntityRef entity, Vec3 half_extents) = 0;
	//@ end

	//@ component Body icon ICON_FA_VOLLEYBALL_BALL id jolt_body
	virtual void initBody(EntityRef entity) = 0;						//@ function alias init
	virtual void addImpulse(EntityRef entity, const Vec3& impulse) = 0; //@ function
	virtual void addForce(EntityRef entity, const Vec3& force) = 0; //@ function
	virtual float getBodySpeed(EntityRef entity) = 0;
	virtual bool isBodyActive(EntityRef entity) = 0;
	virtual bool isDiscreteMotion(EntityRef entity) = 0; //@ getter DescreteMotion
	virtual void setDiscreteMotion(EntityRef entity, bool is_descrete) = 0; //@ setter DescreteMotion
	virtual float getFriction(EntityRef entity) = 0;						//@ getter Friction
	virtual void setFriction(EntityRef entity, float friction) = 0;			//@ setter Friction
	virtual float getRestitution(EntityRef entity) = 0;						//@ getter Restitution
	virtual void setRestitution(EntityRef entity, float restitution) = 0;	//@ setter Restitution
	virtual float getLinearDamping(EntityRef entity) = 0;					//@ getter LinearDamping
	virtual void setLinearDamping(EntityRef entity, float value) = 0;		//@ setter LinearDamping
	virtual float getAngularDamping(EntityRef entity) = 0;					//@ getter AngularDamping
	virtual void setAngularDamping(EntityRef entity, float value) = 0;		//@ setter AngularDamping
	virtual JPH::EMotionType getDynamicType(EntityRef entity) = 0;					//@ getter DynamicType dynenum DynamicType
	virtual void setDynamicType(EntityRef entity, JPH::EMotionType type) = 0;		//@ setter DynamicType
	virtual ObjectLayer getLayer(EntityRef entity) = 0;								//@ getter Layer dynenum Layer
	virtual void setLayer(EntityRef entity, ObjectLayer new_layer) = 0;				//@ setter Layer
	virtual Vec3 getLinearVelocity(EntityRef entity) = 0;							//@ getter LinearVelocity
	virtual void setLinearVelocity(EntityRef entity, const Vec3& velocity) = 0;		//@ setter LinearVelocity
	virtual Vec3 getAngularVelocity(EntityRef entity) = 0;							//@ getter AngularVelocity
	virtual void setAngularVelocity(EntityRef entity, const Vec3& velocity) = 0;	//@ setter AngularVelocity
	//@ end
};

}