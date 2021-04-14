# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/gautam/projects/visual_odom-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gautam/projects/visual_odom-master/build

# Include any dependencies generated for this target.
include src/CMakeFiles/utils.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/utils.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/utils.dir/flags.make

src/CMakeFiles/utils.dir/utils.cpp.o: src/CMakeFiles/utils.dir/flags.make
src/CMakeFiles/utils.dir/utils.cpp.o: ../src/utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gautam/projects/visual_odom-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/utils.dir/utils.cpp.o"
	cd /home/gautam/projects/visual_odom-master/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/utils.dir/utils.cpp.o -c /home/gautam/projects/visual_odom-master/src/utils.cpp

src/CMakeFiles/utils.dir/utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utils.dir/utils.cpp.i"
	cd /home/gautam/projects/visual_odom-master/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gautam/projects/visual_odom-master/src/utils.cpp > CMakeFiles/utils.dir/utils.cpp.i

src/CMakeFiles/utils.dir/utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utils.dir/utils.cpp.s"
	cd /home/gautam/projects/visual_odom-master/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gautam/projects/visual_odom-master/src/utils.cpp -o CMakeFiles/utils.dir/utils.cpp.s

src/CMakeFiles/utils.dir/utils.cpp.o.requires:

.PHONY : src/CMakeFiles/utils.dir/utils.cpp.o.requires

src/CMakeFiles/utils.dir/utils.cpp.o.provides: src/CMakeFiles/utils.dir/utils.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/utils.dir/build.make src/CMakeFiles/utils.dir/utils.cpp.o.provides.build
.PHONY : src/CMakeFiles/utils.dir/utils.cpp.o.provides

src/CMakeFiles/utils.dir/utils.cpp.o.provides.build: src/CMakeFiles/utils.dir/utils.cpp.o


# Object files for target utils
utils_OBJECTS = \
"CMakeFiles/utils.dir/utils.cpp.o"

# External object files for target utils
utils_EXTERNAL_OBJECTS =

src/libutils.so: src/CMakeFiles/utils.dir/utils.cpp.o
src/libutils.so: src/CMakeFiles/utils.dir/build.make
src/libutils.so: src/evaluate/libevaluate_odometry.so
src/libutils.so: src/libfeature.so
src/libutils.so: src/evaluate/libmatrix.so
src/libutils.so: src/libbucket.so
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
src/libutils.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
src/libutils.so: src/CMakeFiles/utils.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gautam/projects/visual_odom-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libutils.so"
	cd /home/gautam/projects/visual_odom-master/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/utils.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/utils.dir/build: src/libutils.so

.PHONY : src/CMakeFiles/utils.dir/build

src/CMakeFiles/utils.dir/requires: src/CMakeFiles/utils.dir/utils.cpp.o.requires

.PHONY : src/CMakeFiles/utils.dir/requires

src/CMakeFiles/utils.dir/clean:
	cd /home/gautam/projects/visual_odom-master/build/src && $(CMAKE_COMMAND) -P CMakeFiles/utils.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/utils.dir/clean

src/CMakeFiles/utils.dir/depend:
	cd /home/gautam/projects/visual_odom-master/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gautam/projects/visual_odom-master /home/gautam/projects/visual_odom-master/src /home/gautam/projects/visual_odom-master/build /home/gautam/projects/visual_odom-master/build/src /home/gautam/projects/visual_odom-master/build/src/CMakeFiles/utils.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/utils.dir/depend

