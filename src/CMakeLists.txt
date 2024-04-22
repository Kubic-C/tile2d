
add_library(tile2d STATIC "base.hpp" "body.hpp" "collide.hpp" "math.hpp" "tileMap.hpp" "world.hpp" "base.cpp" "spatial.hpp")

set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_LIBS   ON)
set(Boost_ROOT "$ENV{BOOST_ROOT}")
find_package(Boost COMPONENTS thread REQUIRED)
target_include_directories(tile2d PUBLIC "${Boost_INCLUDE_DIRS}")
target_link_directories(tile2d PUBLIC "${Boost_LIBRARY_DIRS}")
target_link_libraries(tile2d PUBLIC Boost::thread)

target_include_directories(tile2d PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../")
target_link_libraries(tile2d PUBLIC glm)

add_compile_options("-no-pie")
add_link_options("-no-pie")
