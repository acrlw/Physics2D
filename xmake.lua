set_languages("c++20")
add_rules("mode.debug", "mode.release")

target("Physics2D")
    set_kind("shared")
    add_files("Physics2D/**.cpp")