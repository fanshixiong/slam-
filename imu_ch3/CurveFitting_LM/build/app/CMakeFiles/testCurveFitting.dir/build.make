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
CMAKE_COMMAND = /home/frans/soft/cmake/cmake-3.22.3/bin/cmake

# The command to remove a file.
RM = /home/frans/soft/cmake/cmake-3.22.3/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM/build

# Include any dependencies generated for this target.
include app/CMakeFiles/testCurveFitting.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include app/CMakeFiles/testCurveFitting.dir/compiler_depend.make

# Include the progress variables for this target.
include app/CMakeFiles/testCurveFitting.dir/progress.make

# Include the compile flags for this target's objects.
include app/CMakeFiles/testCurveFitting.dir/flags.make

app/CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.o: app/CMakeFiles/testCurveFitting.dir/flags.make
app/CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.o: ../app/CurveFitting.cpp
app/CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.o: app/CMakeFiles/testCurveFitting.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object app/CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.o"
	cd /data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM/build/app && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT app/CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.o -MF CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.o.d -o CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.o -c /data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM/app/CurveFitting.cpp

app/CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.i"
	cd /data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM/build/app && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM/app/CurveFitting.cpp > CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.i

app/CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.s"
	cd /data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM/build/app && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM/app/CurveFitting.cpp -o CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.s

# Object files for target testCurveFitting
testCurveFitting_OBJECTS = \
"CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.o"

# External object files for target testCurveFitting
testCurveFitting_EXTERNAL_OBJECTS =

app/testCurveFitting: app/CMakeFiles/testCurveFitting.dir/CurveFitting.cpp.o
app/testCurveFitting: app/CMakeFiles/testCurveFitting.dir/build.make
app/testCurveFitting: backend/libslam_course_backend.a
app/testCurveFitting: app/CMakeFiles/testCurveFitting.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable testCurveFitting"
	cd /data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM/build/app && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testCurveFitting.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
app/CMakeFiles/testCurveFitting.dir/build: app/testCurveFitting
.PHONY : app/CMakeFiles/testCurveFitting.dir/build

app/CMakeFiles/testCurveFitting.dir/clean:
	cd /data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM/build/app && $(CMAKE_COMMAND) -P CMakeFiles/testCurveFitting.dir/cmake_clean.cmake
.PHONY : app/CMakeFiles/testCurveFitting.dir/clean

app/CMakeFiles/testCurveFitting.dir/depend:
	cd /data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM /data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM/app /data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM/build /data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM/build/app /data/code/ceres_study/slambook2/imu_ch3/CurveFitting_LM/build/app/CMakeFiles/testCurveFitting.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : app/CMakeFiles/testCurveFitting.dir/depend

