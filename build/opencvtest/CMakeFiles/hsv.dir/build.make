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
CMAKE_SOURCE_DIR = /home/student/myros/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/student/myros/build

# Include any dependencies generated for this target.
include opencvtest/CMakeFiles/hsv.dir/depend.make

# Include the progress variables for this target.
include opencvtest/CMakeFiles/hsv.dir/progress.make

# Include the compile flags for this target's objects.
include opencvtest/CMakeFiles/hsv.dir/flags.make

opencvtest/CMakeFiles/hsv.dir/src/hsv.cpp.o: opencvtest/CMakeFiles/hsv.dir/flags.make
opencvtest/CMakeFiles/hsv.dir/src/hsv.cpp.o: /home/student/myros/src/opencvtest/src/hsv.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/myros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object opencvtest/CMakeFiles/hsv.dir/src/hsv.cpp.o"
	cd /home/student/myros/build/opencvtest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hsv.dir/src/hsv.cpp.o -c /home/student/myros/src/opencvtest/src/hsv.cpp

opencvtest/CMakeFiles/hsv.dir/src/hsv.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hsv.dir/src/hsv.cpp.i"
	cd /home/student/myros/build/opencvtest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/myros/src/opencvtest/src/hsv.cpp > CMakeFiles/hsv.dir/src/hsv.cpp.i

opencvtest/CMakeFiles/hsv.dir/src/hsv.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hsv.dir/src/hsv.cpp.s"
	cd /home/student/myros/build/opencvtest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/myros/src/opencvtest/src/hsv.cpp -o CMakeFiles/hsv.dir/src/hsv.cpp.s

opencvtest/CMakeFiles/hsv.dir/src/hsv.cpp.o.requires:

.PHONY : opencvtest/CMakeFiles/hsv.dir/src/hsv.cpp.o.requires

opencvtest/CMakeFiles/hsv.dir/src/hsv.cpp.o.provides: opencvtest/CMakeFiles/hsv.dir/src/hsv.cpp.o.requires
	$(MAKE) -f opencvtest/CMakeFiles/hsv.dir/build.make opencvtest/CMakeFiles/hsv.dir/src/hsv.cpp.o.provides.build
.PHONY : opencvtest/CMakeFiles/hsv.dir/src/hsv.cpp.o.provides

opencvtest/CMakeFiles/hsv.dir/src/hsv.cpp.o.provides.build: opencvtest/CMakeFiles/hsv.dir/src/hsv.cpp.o


# Object files for target hsv
hsv_OBJECTS = \
"CMakeFiles/hsv.dir/src/hsv.cpp.o"

# External object files for target hsv
hsv_EXTERNAL_OBJECTS =

/home/student/myros/devel/lib/opencvtest/hsv: opencvtest/CMakeFiles/hsv.dir/src/hsv.cpp.o
/home/student/myros/devel/lib/opencvtest/hsv: opencvtest/CMakeFiles/hsv.dir/build.make
/home/student/myros/devel/lib/opencvtest/hsv: /opt/ros/melodic/lib/libcv_bridge.so
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /opt/ros/melodic/lib/libimage_transport.so
/home/student/myros/devel/lib/opencvtest/hsv: /opt/ros/melodic/lib/libmessage_filters.so
/home/student/myros/devel/lib/opencvtest/hsv: /opt/ros/melodic/lib/libclass_loader.so
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/libPocoFoundation.so
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libdl.so
/home/student/myros/devel/lib/opencvtest/hsv: /opt/ros/melodic/lib/libroslib.so
/home/student/myros/devel/lib/opencvtest/hsv: /opt/ros/melodic/lib/librospack.so
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/student/myros/devel/lib/opencvtest/hsv: /opt/ros/melodic/lib/libroscpp.so
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/student/myros/devel/lib/opencvtest/hsv: /opt/ros/melodic/lib/librosconsole.so
/home/student/myros/devel/lib/opencvtest/hsv: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/student/myros/devel/lib/opencvtest/hsv: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/student/myros/devel/lib/opencvtest/hsv: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/student/myros/devel/lib/opencvtest/hsv: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/student/myros/devel/lib/opencvtest/hsv: /opt/ros/melodic/lib/librostime.so
/home/student/myros/devel/lib/opencvtest/hsv: /opt/ros/melodic/lib/libcpp_common.so
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/student/myros/devel/lib/opencvtest/hsv: opencvtest/CMakeFiles/hsv.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/student/myros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/student/myros/devel/lib/opencvtest/hsv"
	cd /home/student/myros/build/opencvtest && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hsv.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
opencvtest/CMakeFiles/hsv.dir/build: /home/student/myros/devel/lib/opencvtest/hsv

.PHONY : opencvtest/CMakeFiles/hsv.dir/build

opencvtest/CMakeFiles/hsv.dir/requires: opencvtest/CMakeFiles/hsv.dir/src/hsv.cpp.o.requires

.PHONY : opencvtest/CMakeFiles/hsv.dir/requires

opencvtest/CMakeFiles/hsv.dir/clean:
	cd /home/student/myros/build/opencvtest && $(CMAKE_COMMAND) -P CMakeFiles/hsv.dir/cmake_clean.cmake
.PHONY : opencvtest/CMakeFiles/hsv.dir/clean

opencvtest/CMakeFiles/hsv.dir/depend:
	cd /home/student/myros/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/myros/src /home/student/myros/src/opencvtest /home/student/myros/build /home/student/myros/build/opencvtest /home/student/myros/build/opencvtest/CMakeFiles/hsv.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : opencvtest/CMakeFiles/hsv.dir/depend

