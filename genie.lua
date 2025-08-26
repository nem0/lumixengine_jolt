if plugin "jolt" then
	files { 
		"external/Jolt/**.c",
		"external/Jolt/**.cpp",
		"external/Jolt/**.h",
		"external/Jolt/**.natvis",
		"src/**.c",
		"src/**.cpp",
		"src/**.h",
		"genie.lua"
	}
	includedirs { "external/" }
	defines { "BUILDING_JOLT", "JPH_DEBUG_RENDERER" }
	dynamic_link_plugin { "engine" }
end