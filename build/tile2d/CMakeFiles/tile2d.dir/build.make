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
include tile2d/CMakeFiles/tile2d.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include tile2d/CMakeFiles/tile2d.dir/compiler_depend.make

# Include the progress variables for this target.
include tile2d/CMakeFiles/tile2d.dir/progress.make

# Include the compile flags for this target's objects.
include tile2d/CMakeFiles/tile2d.dir/flags.make

tile2d/CMakeFiles/tile2d.dir/base.cpp.o: tile2d/CMakeFiles/tile2d.dir/flags.make
tile2d/CMakeFiles/tile2d.dir/base.cpp.o: ../tile2d/base.cpp
tile2d/CMakeFiles/tile2d.dir/base.cpp.o: tile2d/CMakeFiles/tile2d.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sawyer/dev/tile2d/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tile2d/CMakeFiles/tile2d.dir/base.cpp.o"
	cd /home/sawyer/dev/tile2d/build/tile2d && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT tile2d/CMakeFiles/tile2d.dir/base.cpp.o -MF CMakeFiles/tile2d.dir/base.cpp.o.d -o CMakeFiles/tile2d.dir/base.cpp.o -c /home/sawyer/dev/tile2d/tile2d/base.cpp

tile2d/CMakeFiles/tile2d.dir/base.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tile2d.dir/base.cpp.i"
	cd /home/sawyer/dev/tile2d/build/tile2d && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sawyer/dev/tile2d/tile2d/base.cpp > CMakeFiles/tile2d.dir/base.cpp.i

tile2d/CMakeFiles/tile2d.dir/base.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tile2d.dir/base.cpp.s"
	cd /home/sawyer/dev/tile2d/build/tile2d && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sawyer/dev/tile2d/tile2d/base.cpp -o CMakeFiles/tile2d.dir/base.cpp.s

# Object files for target tile2d
tile2d_OBJECTS = \
"CMakeFiles/tile2d.dir/base.cpp.o"

# External object files for target tile2d
tile2d_EXTERNAL_OBJECTS =

../bin/libtile2d.a: tile2d/CMakeFiles/tile2d.dir/base.cpp.o
../bin/libtile2d.a: tile2d/CMakeFiles/tile2d.dir/build.make
../bin/libtile2d.a: tile2d/CMakeFiles/tile2d.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sawyer/dev/tile2d/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../../bin/libtile2d.a"
	cd /home/sawyer/dev/tile2d/build/tile2d && $(CMAKE_COMMAND) -P CMakeFiles/tile2d.dir/cmake_clean_target.cmake
	cd /home/sawyer/dev/tile2d/build/tile2d && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tile2d.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tile2d/CMakeFiles/tile2d.dir/build: ../bin/libtile2d.a
.PHONY : tile2d/CMakeFiles/tile2d.dir/build

tile2d/CMakeFiles/tile2d.dir/clean:
	cd /home/sawyer/dev/tile2d/build/tile2d && $(CMAKE_COMMAND) -P CMakeFiles/tile2d.dir/cmake_clean.cmake
.PHONY : tile2d/CMakeFiles/tile2d.dir/clean

tile2d/CMakeFiles/tile2d.dir/depend:
	cd /home/sawyer/dev/tile2d/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sawyer/dev/tile2d /home/sawyer/dev/tile2d/tile2d /home/sawyer/dev/tile2d/build /home/sawyer/dev/tile2d/build/tile2d /home/sawyer/dev/tile2d/build/tile2d/CMakeFiles/tile2d.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tile2d/CMakeFiles/tile2d.dir/depend

