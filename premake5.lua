solution "QuadPuppet"
    startproject "HoloEdit"

    configurations { "Release", "Debug" }
    platforms { "x86_64" }

    filter "platforms:x86_64"
        architecture "x86_64"

    filter "configurations:Release*"
        defines { "NDEBUG" }
        optimize "Speed"
        symbols "On"

    filter "configurations:Debug*"
        defines { "_DEBUG" }
        optimize "Debug"
        symbols "On"

    filter {}
        
-- Defined as a function so it can be used in other premake scripts.
function QuadPuppetProject(rootDir)
    kind "SharedLib"
    language "C++"
    cppdialect "C++17"
    exceptionhandling "On"
    rtti "On"
    characterset "ASCII"
    location ("build/" .. _ACTION)

    defines {
        "_CRT_SECURE_NO_WARNINGS",
        "_SILENCE_STDEXT_HASH_DEPRECATION_WARNINGS",
        "_USE_MATH_DEFINES",
        "QP_DLL_EXPORT"
    }

    files { 
        path.join(rootDir, "src/*.cpp"),
        path.join(rootDir, "include/**.h"),
        path.join(rootDir, "src/Pinocchio/include/*.h"),
        path.join(rootDir, "src/Pinocchio/src/*.cpp"),
        path.join(rootDir, "src/quadprog/*.cpp"),
        path.join(rootDir, "src/quadprog/*.cc"),
        path.join(rootDir, "src/quadprog_eigen/*.cpp")
    }

    includedirs {
        path.join(rootDir, "include/"),
        path.join(rootDir, "src/Pinocchio/include/"),
        path.join(rootDir, "lib/")
    }

    filter {}
end

project "QuadPuppet"
    QuadPuppetProject("./")