# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/parallels/workspace/KDQ_SLAM/ceres_study

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/parallels/workspace/KDQ_SLAM/ceres_study/build

# Include any dependencies generated for this target.
include src/CMakeFiles/ceres_demo.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/ceres_demo.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/ceres_demo.dir/flags.make

src/CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.o: src/CMakeFiles/ceres_demo.dir/flags.make
src/CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.o: ../src/ceres_demo1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/parallels/workspace/KDQ_SLAM/ceres_study/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.o"
	cd /home/parallels/workspace/KDQ_SLAM/ceres_study/build/src && /usr/bin/g++-5   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.o -c /home/parallels/workspace/KDQ_SLAM/ceres_study/src/ceres_demo1.cpp

src/CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.i"
	cd /home/parallels/workspace/KDQ_SLAM/ceres_study/build/src && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/parallels/workspace/KDQ_SLAM/ceres_study/src/ceres_demo1.cpp > CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.i

src/CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.s"
	cd /home/parallels/workspace/KDQ_SLAM/ceres_study/build/src && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/parallels/workspace/KDQ_SLAM/ceres_study/src/ceres_demo1.cpp -o CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.s

src/CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.o.requires:

.PHONY : src/CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.o.requires

src/CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.o.provides: src/CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/ceres_demo.dir/build.make src/CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.o.provides.build
.PHONY : src/CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.o.provides

src/CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.o.provides.build: src/CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.o


# Object files for target ceres_demo
ceres_demo_OBJECTS = \
"CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.o"

# External object files for target ceres_demo
ceres_demo_EXTERNAL_OBJECTS =

src/ceres_demo: src/CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.o
src/ceres_demo: src/CMakeFiles/ceres_demo.dir/build.make
src/ceres_demo: src/CMakeFiles/ceres_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/parallels/workspace/KDQ_SLAM/ceres_study/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ceres_demo"
	cd /home/parallels/workspace/KDQ_SLAM/ceres_study/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ceres_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/ceres_demo.dir/build: src/ceres_demo

.PHONY : src/CMakeFiles/ceres_demo.dir/build

src/CMakeFiles/ceres_demo.dir/requires: src/CMakeFiles/ceres_demo.dir/ceres_demo1.cpp.o.requires

.PHONY : src/CMakeFiles/ceres_demo.dir/requires

src/CMakeFiles/ceres_demo.dir/clean:
	cd /home/parallels/workspace/KDQ_SLAM/ceres_study/build/src && $(CMAKE_COMMAND) -P CMakeFiles/ceres_demo.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/ceres_demo.dir/clean

src/CMakeFiles/ceres_demo.dir/depend:
	cd /home/parallels/workspace/KDQ_SLAM/ceres_study/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/parallels/workspace/KDQ_SLAM/ceres_study /home/parallels/workspace/KDQ_SLAM/ceres_study/src /home/parallels/workspace/KDQ_SLAM/ceres_study/build /home/parallels/workspace/KDQ_SLAM/ceres_study/build/src /home/parallels/workspace/KDQ_SLAM/ceres_study/build/src/CMakeFiles/ceres_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/ceres_demo.dir/depend

