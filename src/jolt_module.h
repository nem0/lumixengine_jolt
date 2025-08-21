#include "engine/plugin.h"

namespace Lumix {

struct JoltModule : IModule {
	virtual float getSphereRadius(EntityRef entity) const = 0;
	virtual void setSphereRadius(EntityRef entity, float radius) = 0;
	
	virtual Vec3 getBoxHalfExtents(EntityRef entity) const = 0;
	virtual void setBoxHalfExtents(EntityRef entity, Vec3 half_extents) = 0;
};

}