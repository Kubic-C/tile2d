# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sawyer/dev/tile2d

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sawyer/dev/tile2d/build

# Include any dependencies generated for this target.
include deps/glm/glm/CMakeFiles/glm.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include deps/glm/glm/CMakeFiles/glm.dir/compiler_depend.make

# Include the progress variables for this target.
include deps/glm/glm/CMakeFiles/glm.dir/progress.make

# Include the compile flags for this target's objects.
include deps/glm/glm/CMakeFiles/glm.dir/flags.make

deps/glm/glm/CMakeFiles/glm.dir/detail/glm.cpp.o: deps/glm/glm/CMakeFiles/glm.dir/flags.make
deps/glm/glm/CMakeFiles/glm.dir/detail/glm.cpp.o: ../deps/glm/glm/detail/glm.cpp
deps/glm/glm/CMakeFiles/glm.dir/detail/glm.cpp.o: deps/glm/glm/CMakeFiles/glm.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sawyer/dev/tile2d/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object deps/glm/glm/CMakeFiles/glm.dir/detail/glm.cpp.o"
	cd /home/sawyer/dev/tile2d/build/deps/glm/glm && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT deps/glm/glm/CMakeFiles/glm.dir/detail/glm.cpp.o -MF CMakeFiles/glm.dir/detail/glm.cpp.o.d -o CMakeFiles/glm.dir/detail/glm.cpp.o -c /home/sawyer/dev/tile2d/deps/glm/glm/detail/glm.cpp

deps/glm/glm/CMakeFiles/glm.dir/detail/glm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/glm.dir/detail/glm.cpp.i"
	cd /home/sawyer/dev/tile2d/build/deps/glm/glm && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sawyer/dev/tile2d/deps/glm/glm/detail/glm.cpp > CMakeFiles/glm.dir/detail/glm.cpp.i

deps/glm/glm/CMakeFiles/glm.dir/detail/glm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/glm.dir/detail/glm.cpp.s"
	cd /home/sawyer/dev/tile2d/build/deps/glm/glm && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sawyer/dev/tile2d/deps/glm/glm/detail/glm.cpp -o CMakeFiles/glm.dir/detail/glm.cpp.s

# Object files for target glm
glm_OBJECTS = \
"CMakeFiles/glm.dir/detail/glm.cpp.o"

# External object files for target glm
glm_EXTERNAL_OBJECTS =

../bin/libglm.a: deps/glm/glm/CMakeFiles/glm.dir/detail/glm.cpp.o
../bin/libglm.a: deps/glm/glm/CMakeFiles/glm.dir/build.make
../bin/libglm.a: deps/glm/glm/CMakeFiles/glm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sawyer/dev/tile2d/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../../../../bin/libglm.a"
	cd /home/sawyer/dev/tile2d/build/deps/glm/glm && $(CMAKE_COMMAND) -P CMakeFiles/glm.dir/cmake_clean_target.cmake
	cd /home/sawyer/dev/tile2d/build/deps/glm/glm && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/glm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
deps/glm/glm/CMakeFiles/glm.dir/build: ../bin/libglm.a
.PHONY : deps/glm/glm/CMakeFiles/glm.dir/build

deps/glm/glm/CMakeFiles/glm.dir/clean:
	cd /home/sawyer/dev/tile2d/build/deps/glm/glm && $(CMAKE_COMMAND) -P CMakeFiles/glm.dir/cmake_clean.cmake
.PHONY : deps/glm/glm/CMakeFiles/glm.dir/clean

deps/glm/glm/CMakeFiles/glm.dir/depend:
	cd /home/sawyer/dev/tile2d/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sawyer/dev/tile2d /home/sawyer/dev/tile2d/deps/glm/glm /home/sawyer/dev/tile2d/build /home/sawyer/dev/tile2d/build/deps/glm/glm /home/sawyer/dev/tile2d/build/deps/glm/glm/CMakeFiles/glm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : deps/glm/glm/CMakeFiles/glm.dir/depend
