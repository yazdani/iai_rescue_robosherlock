# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yazdani/work/ros/indigo/catkin_ws/src/iai_rescue_robosherlock/nodes

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yazdani/work/ros/indigo/catkin_ws/src/iai_rescue_robosherlock/nodes/build

# Include any dependencies generated for this target.
include CMakeFiles/morpho.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/morpho.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/morpho.dir/flags.make

CMakeFiles/morpho.dir/morpho.cpp.o: CMakeFiles/morpho.dir/flags.make
CMakeFiles/morpho.dir/morpho.cpp.o: ../morpho.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yazdani/work/ros/indigo/catkin_ws/src/iai_rescue_robosherlock/nodes/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/morpho.dir/morpho.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/morpho.dir/morpho.cpp.o -c /home/yazdani/work/ros/indigo/catkin_ws/src/iai_rescue_robosherlock/nodes/morpho.cpp

CMakeFiles/morpho.dir/morpho.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/morpho.dir/morpho.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yazdani/work/ros/indigo/catkin_ws/src/iai_rescue_robosherlock/nodes/morpho.cpp > CMakeFiles/morpho.dir/morpho.cpp.i

CMakeFiles/morpho.dir/morpho.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/morpho.dir/morpho.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yazdani/work/ros/indigo/catkin_ws/src/iai_rescue_robosherlock/nodes/morpho.cpp -o CMakeFiles/morpho.dir/morpho.cpp.s

CMakeFiles/morpho.dir/morpho.cpp.o.requires:
.PHONY : CMakeFiles/morpho.dir/morpho.cpp.o.requires

CMakeFiles/morpho.dir/morpho.cpp.o.provides: CMakeFiles/morpho.dir/morpho.cpp.o.requires
	$(MAKE) -f CMakeFiles/morpho.dir/build.make CMakeFiles/morpho.dir/morpho.cpp.o.provides.build
.PHONY : CMakeFiles/morpho.dir/morpho.cpp.o.provides

CMakeFiles/morpho.dir/morpho.cpp.o.provides.build: CMakeFiles/morpho.dir/morpho.cpp.o

# Object files for target morpho
morpho_OBJECTS = \
"CMakeFiles/morpho.dir/morpho.cpp.o"

# External object files for target morpho
morpho_EXTERNAL_OBJECTS =

morpho: CMakeFiles/morpho.dir/morpho.cpp.o
morpho: CMakeFiles/morpho.dir/build.make
morpho: /usr/lib/x86_64-linux-gnu/libboost_system.so
morpho: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
morpho: /usr/lib/x86_64-linux-gnu/libboost_thread.so
morpho: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
morpho: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
morpho: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
morpho: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
morpho: /usr/lib/x86_64-linux-gnu/libpthread.so
morpho: /usr/local/lib/libpcl_common.so
morpho: /usr/local/lib/libpcl_octree.so
morpho: /usr/lib/libOpenNI.so
morpho: /usr/lib/libvtkCommon.so.5.8.0
morpho: /usr/lib/libvtkFiltering.so.5.8.0
morpho: /usr/lib/libvtkImaging.so.5.8.0
morpho: /usr/lib/libvtkGraphics.so.5.8.0
morpho: /usr/lib/libvtkGenericFiltering.so.5.8.0
morpho: /usr/lib/libvtkIO.so.5.8.0
morpho: /usr/lib/libvtkRendering.so.5.8.0
morpho: /usr/lib/libvtkVolumeRendering.so.5.8.0
morpho: /usr/lib/libvtkHybrid.so.5.8.0
morpho: /usr/lib/libvtkWidgets.so.5.8.0
morpho: /usr/lib/libvtkParallel.so.5.8.0
morpho: /usr/lib/libvtkInfovis.so.5.8.0
morpho: /usr/lib/libvtkGeovis.so.5.8.0
morpho: /usr/lib/libvtkViews.so.5.8.0
morpho: /usr/lib/libvtkCharts.so.5.8.0
morpho: /usr/local/lib/libpcl_io.so
morpho: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
morpho: /usr/local/lib/libpcl_kdtree.so
morpho: /usr/local/lib/libpcl_search.so
morpho: /usr/local/lib/libpcl_sample_consensus.so
morpho: /usr/local/lib/libpcl_filters.so
morpho: /usr/local/lib/libpcl_features.so
morpho: /usr/local/lib/libpcl_registration.so
morpho: /usr/local/lib/libpcl_recognition.so
morpho: /usr/local/lib/libpcl_keypoints.so
morpho: /usr/lib/x86_64-linux-gnu/libqhull.so
morpho: /usr/local/lib/libpcl_surface.so
morpho: /usr/local/lib/libpcl_segmentation.so
morpho: /usr/local/lib/libpcl_visualization.so
morpho: /usr/local/lib/libpcl_people.so
morpho: /usr/local/lib/libpcl_outofcore.so
morpho: /usr/local/lib/libpcl_tracking.so
morpho: /usr/lib/x86_64-linux-gnu/libboost_system.so
morpho: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
morpho: /usr/lib/x86_64-linux-gnu/libboost_thread.so
morpho: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
morpho: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
morpho: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
morpho: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
morpho: /usr/lib/x86_64-linux-gnu/libpthread.so
morpho: /usr/lib/x86_64-linux-gnu/libqhull.so
morpho: /usr/lib/libOpenNI.so
morpho: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
morpho: /usr/lib/libvtkCommon.so.5.8.0
morpho: /usr/lib/libvtkFiltering.so.5.8.0
morpho: /usr/lib/libvtkImaging.so.5.8.0
morpho: /usr/lib/libvtkGraphics.so.5.8.0
morpho: /usr/lib/libvtkGenericFiltering.so.5.8.0
morpho: /usr/lib/libvtkIO.so.5.8.0
morpho: /usr/lib/libvtkRendering.so.5.8.0
morpho: /usr/lib/libvtkVolumeRendering.so.5.8.0
morpho: /usr/lib/libvtkHybrid.so.5.8.0
morpho: /usr/lib/libvtkWidgets.so.5.8.0
morpho: /usr/lib/libvtkParallel.so.5.8.0
morpho: /usr/lib/libvtkInfovis.so.5.8.0
morpho: /usr/lib/libvtkGeovis.so.5.8.0
morpho: /usr/lib/libvtkViews.so.5.8.0
morpho: /usr/lib/libvtkCharts.so.5.8.0
morpho: /usr/local/lib/libpcl_common.so
morpho: /usr/local/lib/libpcl_octree.so
morpho: /usr/local/lib/libpcl_io.so
morpho: /usr/local/lib/libpcl_kdtree.so
morpho: /usr/local/lib/libpcl_search.so
morpho: /usr/local/lib/libpcl_sample_consensus.so
morpho: /usr/local/lib/libpcl_filters.so
morpho: /usr/local/lib/libpcl_features.so
morpho: /usr/local/lib/libpcl_registration.so
morpho: /usr/local/lib/libpcl_recognition.so
morpho: /usr/local/lib/libpcl_keypoints.so
morpho: /usr/local/lib/libpcl_surface.so
morpho: /usr/local/lib/libpcl_segmentation.so
morpho: /usr/local/lib/libpcl_visualization.so
morpho: /usr/local/lib/libpcl_people.so
morpho: /usr/local/lib/libpcl_outofcore.so
morpho: /usr/local/lib/libpcl_tracking.so
morpho: /usr/lib/libvtkViews.so.5.8.0
morpho: /usr/lib/libvtkInfovis.so.5.8.0
morpho: /usr/lib/libvtkWidgets.so.5.8.0
morpho: /usr/lib/libvtkVolumeRendering.so.5.8.0
morpho: /usr/lib/libvtkHybrid.so.5.8.0
morpho: /usr/lib/libvtkParallel.so.5.8.0
morpho: /usr/lib/libvtkRendering.so.5.8.0
morpho: /usr/lib/libvtkImaging.so.5.8.0
morpho: /usr/lib/libvtkGraphics.so.5.8.0
morpho: /usr/lib/libvtkIO.so.5.8.0
morpho: /usr/lib/libvtkFiltering.so.5.8.0
morpho: /usr/lib/libvtkCommon.so.5.8.0
morpho: /usr/lib/libvtksys.so.5.8.0
morpho: CMakeFiles/morpho.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable morpho"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/morpho.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/morpho.dir/build: morpho
.PHONY : CMakeFiles/morpho.dir/build

CMakeFiles/morpho.dir/requires: CMakeFiles/morpho.dir/morpho.cpp.o.requires
.PHONY : CMakeFiles/morpho.dir/requires

CMakeFiles/morpho.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/morpho.dir/cmake_clean.cmake
.PHONY : CMakeFiles/morpho.dir/clean

CMakeFiles/morpho.dir/depend:
	cd /home/yazdani/work/ros/indigo/catkin_ws/src/iai_rescue_robosherlock/nodes/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yazdani/work/ros/indigo/catkin_ws/src/iai_rescue_robosherlock/nodes /home/yazdani/work/ros/indigo/catkin_ws/src/iai_rescue_robosherlock/nodes /home/yazdani/work/ros/indigo/catkin_ws/src/iai_rescue_robosherlock/nodes/build /home/yazdani/work/ros/indigo/catkin_ws/src/iai_rescue_robosherlock/nodes/build /home/yazdani/work/ros/indigo/catkin_ws/src/iai_rescue_robosherlock/nodes/build/CMakeFiles/morpho.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/morpho.dir/depend

