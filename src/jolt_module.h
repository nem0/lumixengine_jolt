#include "engine/plugin.h"

namespace Lumix {

struct JoltModule : IModule {
	virtual u32 getSphereShapeCount(EntityRef entity) const = 0;
	virtual float getSphereRadius(EntityRef entity, u32 idx) const = 0;
	virtual Vec3 getSphereOffsetPosition(EntityRef entity, u32 idx) const = 0;
	
	virtual u32 getBoxShapeCount(EntityRef entity) const = 0;
	virtual Vec3 getBoxHalfExtents(EntityRef entity, u32 idx) const = 0;
	virtual Vec3 getBoxOffsetPosition(EntityRef entity, u32 idx) const = 0;
	virtual Quat getBoxOffsetRotationQuat(EntityRef entity, u32 idx) const = 0;
};

}