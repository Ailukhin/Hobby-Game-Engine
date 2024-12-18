workspace "Hobby Game Engine"
   architecture "x64"
   configurations { "Debug", "Release", "Dist" }
   startproject "Game App"

   -- Workspace-wide build options for MSVC
   filter "system:windows"
      buildoptions { "/EHsc", "/Zc:preprocessor", "/Zc:__cplusplus" }

OutputDir = "%{cfg.system}-%{cfg.architecture}/%{cfg.buildcfg}"

group "Core"
	include "Game Engine Core/Build-Core.lua"
group ""

include "Game App/Build-App.lua"