
add_executable(tile2d "main.cpp" "physicsAlgorithims.hpp")

set(Boost_USE_STATIC_LIBS   ON)
set(Boost_ROOT "$ENV{BOOST_ROOT}")
find_package(Boost REQUIRED)
target_include_directories(tile2d PUBLIC "${Boost_INCLUDE_DIRS}")
target_link_directories(tile2d PUBLIC "${Boost_LIBRARY_DIRS}")

target_link_libraries(tile2d PUBLIC sfml-system
sfml-window
sfml-network
sfml-graphics
sfml-audio
glm
THST)