#include "core/allocator.h"
#include "core/color.h"
#include "core/geometry.h"
#include "core/profiler.h"
#include "core/string.h"
#include "editor/action.h"
#include "editor/studio_app.h"
#include "editor/world_editor.h"
#include "engine/component_uid.h"
#include "engine/world.h"
#include "../jolt_module.h"

namespace Lumix {

static const ComponentType JOLT_BOX_TYPE = reflection::getComponentType("jolt_box");
static const ComponentType JOLT_SPHERE_TYPE = reflection::getComponentType("jolt_sphere");

struct StudioAppPlugin : StudioApp::IPlugin {
	StudioAppPlugin(StudioApp& app) : m_app(app) {}
	
	void init() override {}
	const char* getName() const override { return "jolt"; }
	
	void update(float time_delta) override {
		if (m_app.checkShortcut(m_toggle_debug_draw_action, true)) {
			JoltModule* jolt = (JoltModule*)m_app.getWorldEditor().getWorld()->getModule("jolt");
			jolt->toggleDebugDraw();
		}
	}

	static void addSphere(WorldView& view, const DVec3& center, float radius, Color color) {
		static const int COLS = 36;
		static const int ROWS = COLS >> 1;
		static const float STEP = (PI / 180.0f) * 360.0f / COLS;
		int p2 = COLS >> 1;
		int r2 = ROWS >> 1;
		float prev_ci = 1;
		float prev_si = 0;

		const u32 count = 2 * r2 * 2 * p2;
		WorldView::Vertex* vertices = view.render(true, count * 6);
		const DVec3& cam_pos = view.getViewport().pos;
		auto add_line = [&](const DVec3& a, const DVec3& b){
			vertices[0].pos = Vec3(a - cam_pos);
			vertices[1].pos = Vec3(b - cam_pos);
			vertices[0].abgr = color.abgr();
			vertices[1].abgr = color.abgr();
			vertices += 2;
		};

		for (int y = -r2; y < r2; ++y) {
			float cy = cosf(y * STEP);
			float cy1 = cosf((y + 1) * STEP);
			float sy = sinf(y * STEP);
			float sy1 = sinf((y + 1) * STEP);

			for (int i = -p2; i < p2; ++i) {
				float ci = cosf(i * STEP);
				float si = sinf(i * STEP);
				add_line(DVec3(center.x + radius * ci * cy, center.y + radius * sy, center.z + radius * si * cy),
						DVec3(center.x + radius * ci * cy1, center.y + radius * sy1, center.z + radius * si * cy1));
				add_line(DVec3(center.x + radius * ci * cy, center.y + radius * sy, center.z + radius * si * cy),
						DVec3(center.x + radius * prev_ci * cy, center.y + radius * sy, center.z + radius * prev_si * cy));
				add_line(DVec3(center.x + radius * prev_ci * cy1, center.y + radius * sy1, center.z + radius * prev_si * cy1),
						DVec3(center.x + radius * ci * cy1, center.y + radius * sy1, center.z + radius * si * cy1));
				prev_ci = ci;
				prev_si = si;
			}
		}
	}

	static void addCube(WorldView& view, const DVec3& pos, const Vec3& right, const Vec3& up, const Vec3& dir, Color color) {
		WorldView::Vertex* vertices = view.render(true, 24);
		const DVec3& cam_pos = view.getViewport().pos;

		auto add_line = [&](const DVec3& a, const DVec3& b){
			vertices[0].pos = Vec3(a - cam_pos);
			vertices[1].pos = Vec3(b - cam_pos);
			vertices[0].abgr = color.abgr();
			vertices[1].abgr = color.abgr();
			vertices += 2;
		};

		add_line(pos + dir + up + right, pos + dir + up - right);
		add_line(pos - dir + up + right, pos - dir + up - right);
		add_line(pos + dir + up + right, pos - dir + up + right);
		add_line(pos + dir + up - right, pos - dir + up - right);

		add_line(pos + dir - up + right, pos + dir - up - right);
		add_line(pos - dir - up + right, pos - dir - up - right);
		add_line(pos + dir - up + right, pos - dir - up + right);
		add_line(pos + dir - up - right, pos - dir - up - right);

		add_line(pos + dir + up + right, pos + dir - up + right);
		add_line(pos + dir + up - right, pos + dir - up - right);
		add_line(pos - dir + up + right, pos - dir - up + right);
		add_line(pos - dir + up - right, pos - dir - up - right);
	}

	bool showGizmo(struct WorldView& view, ComponentUID cmp) override {
		if (cmp.type == JOLT_SPHERE_TYPE) {
			auto* module = (JoltModule*)cmp.module;
			const Transform tr = module->getWorld().getTransform(*cmp.entity);
			const float radius = module->getSphereRadius(*cmp.entity);
			addSphere(view, tr.pos, radius, Color::GREEN);
			return true;
		}

		if (cmp.type == JOLT_BOX_TYPE) {
			auto* module = (JoltModule*)cmp.module;
			const Transform tr = module->getWorld().getTransform(*cmp.entity);
			const Vec3 half = module->getBoxHalfExtents(*cmp.entity);

			addCube(view
				, tr.pos
				, tr.rot.rotate(Vec3(half.x, 0, 0))
				, tr.rot.rotate(Vec3(0, half.y, 0))
				, tr.rot.rotate(Vec3(0, 0, half.z))
				, Color::BLUE);

			return true;
		}

		return false;
	}

	StudioApp& m_app;
	Action m_toggle_debug_draw_action{"Jolt", "Debug draw", "Toggle debug draw", "jolt_debug_draw", ""};
};

LUMIX_STUDIO_ENTRY(jolt)
{
	PROFILE_FUNCTION();
	IAllocator& allocator = app.getAllocator();
	return LUMIX_NEW(allocator, StudioAppPlugin)(app);
}

}