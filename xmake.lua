add_rules("mode.debug", "mode.release", "mode.profile")
set_languages("c++20")

target("Physics2D")
    if is_mode("debug") then 
        set_kind("static")
        add_defines("DEBUG")
        set_symbols("debug")
        set_optimize("none")
    end 
    if is_mode("release", "profile") then 
        set_kind("shared")
        add_defines("NDEBUG")
        set_symbols("hidden")
        set_optimize("fastest")
    end

    add_headerfiles("Physics2D/include/*.h")
    add_includedirs("Physics2D/include")
    add_files("Physics2D/source/collision/*.cpp")
    add_files("Physics2D/source/dynamics/*.cpp")
    add_files("Physics2D/source/math/*.cpp")
    add_files("Physics2D/source/other/*.cpp")