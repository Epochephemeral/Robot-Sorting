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
include dobot/CMakeFiles/DobotClient_JOG.dir/depend.make

# Include the progress variables for this target.
include dobot/CMakeFiles/DobotClient_JOG.dir/progress.make

# Include the compile flags for this target's objects.
include dobot/CMakeFiles/DobotClient_JOG.dir/flags.make

dobot/CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.o: dobot/CMakeFiles/DobotClient_JOG.dir/flags.make
dobot/CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.o: /home/student/myros/src/dobot/src/DobotClient_JOG.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/myros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object dobot/CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.o"
	cd /home/student/myros/build/dobot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.o -c /home/student/myros/src/dobot/src/DobotClient_JOG.cpp

dobot/CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.i"
	cd /home/student/myros/build/dobot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/myros/src/dobot/src/DobotClient_JOG.cpp > CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.i

dobot/CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.s"
	cd /home/student/myros/build/dobot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/myros/src/dobot/src/DobotClient_JOG.cpp -o CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.s

dobot/CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.o.requires:

.PHONY : dobot/CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.o.requires

dobot/CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.o.provides: dobot/CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.o.requires
	$(MAKE) -f dobot/CMakeFiles/DobotClient_JOG.dir/build.make dobot/CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.o.provides.build
.PHONY : dobot/CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.o.provides

dobot/CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.o.provides.build: dobot/CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.o


# Object files for target DobotClient_JOG
DobotClient_JOG_OBJECTS = \
"CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.o"

# External object files for target DobotClient_JOG
DobotClient_JOG_EXTERNAL_OBJECTS =

/home/student/myros/devel/lib/dobot/DobotClient_JOG: dobot/CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.o
/home/student/myros/devel/lib/dobot/DobotClient_JOG: dobot/CMakeFiles/DobotClient_JOG.dir/build.make
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /opt/ros/melodic/lib/libroscpp.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /opt/ros/melodic/lib/librosconsole.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /opt/ros/melodic/lib/librostime.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /opt/ros/melodic/lib/libcpp_common.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/student/myros/devel/lib/dobot/DobotClient_JOG: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/student/myros/devel/lib/dobot/DobotClient_JOG: dobot/CMakeFiles/DobotClient_JOG.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/student/myros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/student/myros/devel/lib/dobot/DobotClient_JOG"
	cd /home/student/myros/build/dobot && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/DobotClient_JOG.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
dobot/CMakeFiles/DobotClient_JOG.dir/build: /home/student/myros/devel/lib/dobot/DobotClient_JOG

.PHONY : dobot/CMakeFiles/DobotClient_JOG.dir/build

dobot/CMakeFiles/DobotClient_JOG.dir/requires: dobot/CMakeFiles/DobotClient_JOG.dir/src/DobotClient_JOG.cpp.o.requires

.PHONY : dobot/CMakeFiles/DobotClient_JOG.dir/requires

dobot/CMakeFiles/DobotClient_JOG.dir/clean:
	cd /home/student/myros/build/dobot && $(CMAKE_COMMAND) -P CMakeFiles/DobotClient_JOG.dir/cmake_clean.cmake
.PHONY : dobot/CMakeFiles/DobotClient_JOG.dir/clean

dobot/CMakeFiles/DobotClient_JOG.dir/depend:
	cd /home/student/myros/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/myros/src /home/student/myros/src/dobot /home/student/myros/build /home/student/myros/build/dobot /home/student/myros/build/dobot/CMakeFiles/DobotClient_JOG.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : dobot/CMakeFiles/DobotClient_JOG.dir/depend

