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
CMAKE_SOURCE_DIR = /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/bin

# Include any dependencies generated for this target.
include CMakeFiles/data_gen.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/data_gen.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/data_gen.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/data_gen.dir/flags.make

CMakeFiles/data_gen.dir/main/gener_alldata.cpp.o: CMakeFiles/data_gen.dir/flags.make
CMakeFiles/data_gen.dir/main/gener_alldata.cpp.o: ../main/gener_alldata.cpp
CMakeFiles/data_gen.dir/main/gener_alldata.cpp.o: CMakeFiles/data_gen.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/data_gen.dir/main/gener_alldata.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/data_gen.dir/main/gener_alldata.cpp.o -MF CMakeFiles/data_gen.dir/main/gener_alldata.cpp.o.d -o CMakeFiles/data_gen.dir/main/gener_alldata.cpp.o -c /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/main/gener_alldata.cpp

CMakeFiles/data_gen.dir/main/gener_alldata.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data_gen.dir/main/gener_alldata.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/main/gener_alldata.cpp > CMakeFiles/data_gen.dir/main/gener_alldata.cpp.i

CMakeFiles/data_gen.dir/main/gener_alldata.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data_gen.dir/main/gener_alldata.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/main/gener_alldata.cpp -o CMakeFiles/data_gen.dir/main/gener_alldata.cpp.s

CMakeFiles/data_gen.dir/src/param.cpp.o: CMakeFiles/data_gen.dir/flags.make
CMakeFiles/data_gen.dir/src/param.cpp.o: ../src/param.cpp
CMakeFiles/data_gen.dir/src/param.cpp.o: CMakeFiles/data_gen.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/data_gen.dir/src/param.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/data_gen.dir/src/param.cpp.o -MF CMakeFiles/data_gen.dir/src/param.cpp.o.d -o CMakeFiles/data_gen.dir/src/param.cpp.o -c /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/src/param.cpp

CMakeFiles/data_gen.dir/src/param.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data_gen.dir/src/param.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/src/param.cpp > CMakeFiles/data_gen.dir/src/param.cpp.i

CMakeFiles/data_gen.dir/src/param.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data_gen.dir/src/param.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/src/param.cpp -o CMakeFiles/data_gen.dir/src/param.cpp.s

CMakeFiles/data_gen.dir/src/utilities.cpp.o: CMakeFiles/data_gen.dir/flags.make
CMakeFiles/data_gen.dir/src/utilities.cpp.o: ../src/utilities.cpp
CMakeFiles/data_gen.dir/src/utilities.cpp.o: CMakeFiles/data_gen.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/data_gen.dir/src/utilities.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/data_gen.dir/src/utilities.cpp.o -MF CMakeFiles/data_gen.dir/src/utilities.cpp.o.d -o CMakeFiles/data_gen.dir/src/utilities.cpp.o -c /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/src/utilities.cpp

CMakeFiles/data_gen.dir/src/utilities.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data_gen.dir/src/utilities.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/src/utilities.cpp > CMakeFiles/data_gen.dir/src/utilities.cpp.i

CMakeFiles/data_gen.dir/src/utilities.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data_gen.dir/src/utilities.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/src/utilities.cpp -o CMakeFiles/data_gen.dir/src/utilities.cpp.s

CMakeFiles/data_gen.dir/src/imu.cpp.o: CMakeFiles/data_gen.dir/flags.make
CMakeFiles/data_gen.dir/src/imu.cpp.o: ../src/imu.cpp
CMakeFiles/data_gen.dir/src/imu.cpp.o: CMakeFiles/data_gen.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/data_gen.dir/src/imu.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/data_gen.dir/src/imu.cpp.o -MF CMakeFiles/data_gen.dir/src/imu.cpp.o.d -o CMakeFiles/data_gen.dir/src/imu.cpp.o -c /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/src/imu.cpp

CMakeFiles/data_gen.dir/src/imu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data_gen.dir/src/imu.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/src/imu.cpp > CMakeFiles/data_gen.dir/src/imu.cpp.i

CMakeFiles/data_gen.dir/src/imu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data_gen.dir/src/imu.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/src/imu.cpp -o CMakeFiles/data_gen.dir/src/imu.cpp.s

# Object files for target data_gen
data_gen_OBJECTS = \
"CMakeFiles/data_gen.dir/main/gener_alldata.cpp.o" \
"CMakeFiles/data_gen.dir/src/param.cpp.o" \
"CMakeFiles/data_gen.dir/src/utilities.cpp.o" \
"CMakeFiles/data_gen.dir/src/imu.cpp.o"

# External object files for target data_gen
data_gen_EXTERNAL_OBJECTS =

data_gen: CMakeFiles/data_gen.dir/main/gener_alldata.cpp.o
data_gen: CMakeFiles/data_gen.dir/src/param.cpp.o
data_gen: CMakeFiles/data_gen.dir/src/utilities.cpp.o
data_gen: CMakeFiles/data_gen.dir/src/imu.cpp.o
data_gen: CMakeFiles/data_gen.dir/build.make
data_gen: /usr/local/lib/libopencv_stitching.so.3.4.1
data_gen: /usr/local/lib/libopencv_superres.so.3.4.1
data_gen: /usr/local/lib/libopencv_videostab.so.3.4.1
data_gen: /usr/local/lib/libopencv_aruco.so.3.4.1
data_gen: /usr/local/lib/libopencv_bgsegm.so.3.4.1
data_gen: /usr/local/lib/libopencv_bioinspired.so.3.4.1
data_gen: /usr/local/lib/libopencv_ccalib.so.3.4.1
data_gen: /usr/local/lib/libopencv_cvv.so.3.4.1
data_gen: /usr/local/lib/libopencv_dnn_objdetect.so.3.4.1
data_gen: /usr/local/lib/libopencv_dpm.so.3.4.1
data_gen: /usr/local/lib/libopencv_face.so.3.4.1
data_gen: /usr/local/lib/libopencv_freetype.so.3.4.1
data_gen: /usr/local/lib/libopencv_fuzzy.so.3.4.1
data_gen: /usr/local/lib/libopencv_hdf.so.3.4.1
data_gen: /usr/local/lib/libopencv_hfs.so.3.4.1
data_gen: /usr/local/lib/libopencv_img_hash.so.3.4.1
data_gen: /usr/local/lib/libopencv_line_descriptor.so.3.4.1
data_gen: /usr/local/lib/libopencv_optflow.so.3.4.1
data_gen: /usr/local/lib/libopencv_reg.so.3.4.1
data_gen: /usr/local/lib/libopencv_rgbd.so.3.4.1
data_gen: /usr/local/lib/libopencv_saliency.so.3.4.1
data_gen: /usr/local/lib/libopencv_stereo.so.3.4.1
data_gen: /usr/local/lib/libopencv_structured_light.so.3.4.1
data_gen: /usr/local/lib/libopencv_surface_matching.so.3.4.1
data_gen: /usr/local/lib/libopencv_tracking.so.3.4.1
data_gen: /usr/local/lib/libopencv_xfeatures2d.so.3.4.1
data_gen: /usr/local/lib/libopencv_ximgproc.so.3.4.1
data_gen: /usr/local/lib/libopencv_xobjdetect.so.3.4.1
data_gen: /usr/local/lib/libopencv_xphoto.so.3.4.1
data_gen: /usr/local/lib/libopencv_shape.so.3.4.1
data_gen: /usr/local/lib/libopencv_photo.so.3.4.1
data_gen: /usr/local/lib/libopencv_datasets.so.3.4.1
data_gen: /usr/local/lib/libopencv_plot.so.3.4.1
data_gen: /usr/local/lib/libopencv_text.so.3.4.1
data_gen: /usr/local/lib/libopencv_dnn.so.3.4.1
data_gen: /usr/local/lib/libopencv_ml.so.3.4.1
data_gen: /usr/local/lib/libopencv_video.so.3.4.1
data_gen: /usr/local/lib/libopencv_calib3d.so.3.4.1
data_gen: /usr/local/lib/libopencv_features2d.so.3.4.1
data_gen: /usr/local/lib/libopencv_highgui.so.3.4.1
data_gen: /usr/local/lib/libopencv_videoio.so.3.4.1
data_gen: /usr/local/lib/libopencv_viz.so.3.4.1
data_gen: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.1
data_gen: /usr/local/lib/libopencv_flann.so.3.4.1
data_gen: /usr/local/lib/libopencv_imgcodecs.so.3.4.1
data_gen: /usr/local/lib/libopencv_objdetect.so.3.4.1
data_gen: /usr/local/lib/libopencv_imgproc.so.3.4.1
data_gen: /usr/local/lib/libopencv_core.so.3.4.1
data_gen: CMakeFiles/data_gen.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable data_gen"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/data_gen.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/data_gen.dir/build: data_gen
.PHONY : CMakeFiles/data_gen.dir/build

CMakeFiles/data_gen.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/data_gen.dir/cmake_clean.cmake
.PHONY : CMakeFiles/data_gen.dir/clean

CMakeFiles/data_gen.dir/depend:
	cd /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/bin && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/bin /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/bin /data/code/ceres_study/slambook2/imu_ch2/vio_data_simulation-master/bin/CMakeFiles/data_gen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/data_gen.dir/depend

