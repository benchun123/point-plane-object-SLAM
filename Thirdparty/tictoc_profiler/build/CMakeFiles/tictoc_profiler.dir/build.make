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
CMAKE_SOURCE_DIR = /home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/build

# Include any dependencies generated for this target.
include CMakeFiles/tictoc_profiler.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/tictoc_profiler.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tictoc_profiler.dir/flags.make

CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.o: CMakeFiles/tictoc_profiler.dir/flags.make
CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.o: ../src/profiler.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.o -c /home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/src/profiler.cpp

CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/src/profiler.cpp > CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.i

CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/src/profiler.cpp -o CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.s

CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.o.requires:

.PHONY : CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.o.requires

CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.o.provides: CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.o.requires
	$(MAKE) -f CMakeFiles/tictoc_profiler.dir/build.make CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.o.provides.build
.PHONY : CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.o.provides

CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.o.provides.build: CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.o


# Object files for target tictoc_profiler
tictoc_profiler_OBJECTS = \
"CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.o"

# External object files for target tictoc_profiler
tictoc_profiler_EXTERNAL_OBJECTS =

../lib/libtictoc_profiler.so: CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.o
../lib/libtictoc_profiler.so: CMakeFiles/tictoc_profiler.dir/build.make
../lib/libtictoc_profiler.so: /opt/ros/melodic/lib/libroscpp.so
../lib/libtictoc_profiler.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../lib/libtictoc_profiler.so: /opt/ros/melodic/lib/librosconsole.so
../lib/libtictoc_profiler.so: /opt/ros/melodic/lib/librosconsole_log4cxx.so
../lib/libtictoc_profiler.so: /opt/ros/melodic/lib/librosconsole_backend_interface.so
../lib/libtictoc_profiler.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
../lib/libtictoc_profiler.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../lib/libtictoc_profiler.so: /opt/ros/melodic/lib/libxmlrpcpp.so
../lib/libtictoc_profiler.so: /opt/ros/melodic/lib/libroscpp_serialization.so
../lib/libtictoc_profiler.so: /opt/ros/melodic/lib/librostime.so
../lib/libtictoc_profiler.so: /opt/ros/melodic/lib/libcpp_common.so
../lib/libtictoc_profiler.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
../lib/libtictoc_profiler.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../lib/libtictoc_profiler.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../lib/libtictoc_profiler.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../lib/libtictoc_profiler.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../lib/libtictoc_profiler.so: /usr/lib/x86_64-linux-gnu/libpthread.so
../lib/libtictoc_profiler.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
../lib/libtictoc_profiler.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
../lib/libtictoc_profiler.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../lib/libtictoc_profiler.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../lib/libtictoc_profiler.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../lib/libtictoc_profiler.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../lib/libtictoc_profiler.so: /usr/lib/x86_64-linux-gnu/libpthread.so
../lib/libtictoc_profiler.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
../lib/libtictoc_profiler.so: CMakeFiles/tictoc_profiler.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library ../lib/libtictoc_profiler.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tictoc_profiler.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/tictoc_profiler.dir/build: ../lib/libtictoc_profiler.so

.PHONY : CMakeFiles/tictoc_profiler.dir/build

CMakeFiles/tictoc_profiler.dir/requires: CMakeFiles/tictoc_profiler.dir/src/profiler.cpp.o.requires

.PHONY : CMakeFiles/tictoc_profiler.dir/requires

CMakeFiles/tictoc_profiler.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tictoc_profiler.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tictoc_profiler.dir/clean

CMakeFiles/tictoc_profiler.dir/depend:
	cd /home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler /home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler /home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/build /home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/build /home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/build/CMakeFiles/tictoc_profiler.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tictoc_profiler.dir/depend
