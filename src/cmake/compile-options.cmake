function(ics_set_compile_options target)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(${target} PUBLIC -pedantic -Wall -Weverything -Wno-float-equal -Wno-padded -Wno-c++98-compat -Wno-c++98-compat-pedantic -Wno-missing-prototypes -Wno-range-loop-analysis)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${target} PUBLIC -pedantic -Wall -Wextra -Wno-float-equal -Wno-padded -Wno-unused-parameter -Wcast-align -Wcast-qual -Wctor-dtor-privacy -Wdisabled-optimization -Wformat=2 -Winit-self -Wlogical-op -Wmissing-declarations -Wmissing-include-dirs -Wnoexcept -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow -Wsign-conversion -Wsign-promo -Wstrict-null-sentinel -Wstrict-overflow=5 -Wswitch-default -Wundef)
  endif()

  if(CONFIG STREQUAL "Debug")
    target_compile_options(${target} PUBLIC -O0 -g3)
  elseif(CONFIG STREQUAL "Release")
    target_compile_options(${target} PUBLIC -O3)
  endif()
endfunction()
