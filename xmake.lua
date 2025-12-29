set_project("smb-engine")
set_version("0.1.0")

set_languages("c++17")

set_warnings("all", "extra")

add_rules("mode.debug", "mode.release")

if is_mode("release") then
    set_optimize("fastest")
    add_defines("NDEBUG")
end

if is_mode("debug") then
    set_symbols("debug")
    set_optimize("none")
end

add_requires("raylib 5.5")

-- Main engine executable
target("smb-engine")
    set_kind("binary")
    add_files("src/**.cpp")
    add_includedirs("src")
    add_packages("raylib")

    if is_plat("windows") then
        add_syslinks("winmm", "gdi32", "opengl32")
    elseif is_plat("linux") then
        add_syslinks("pthread", "dl", "m")
    elseif is_plat("macosx") then
        add_frameworks("CoreVideo", "IOKit", "Cocoa", "GLUT", "OpenGL")
    end

-- Unit tests
target("tests")
    set_kind("binary")
    set_default(false)

    -- Test files
    add_files("tests/**.cpp")
    add_includedirs("tests")
    add_includedirs("src")

    -- Include source files needed by tests (excluding main.cpp)
    add_files("src/platform/platform.cpp")
    add_files("src/game/game_loop.cpp")
    add_files("src/physics/ball_physics.cpp")

    if is_plat("windows") then
        add_syslinks("winmm")
    elseif is_plat("linux") then
        add_syslinks("pthread", "dl", "m")
    end
