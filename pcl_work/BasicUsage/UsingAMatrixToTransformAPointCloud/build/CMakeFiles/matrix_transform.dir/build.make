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
CMAKE_SOURCE_DIR = /home/dlonng/Documents/pcl_work/BasicUsage/UsingAMatrixToTransformAPointCloud

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dlonng/Documents/pcl_work/BasicUsage/UsingAMatrixToTransformAPointCloud/build

# Include any dependencies generated for this target.
include CMakeFiles/matrix_transform.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/matrix_transform.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/matrix_transform.dir/flags.make

CMakeFiles/matrix_transform.dir/matrix_transform.cpp.o: CMakeFiles/matrix_transform.dir/flags.make
CMakeFiles/matrix_transform.dir/matrix_transform.cpp.o: ../matrix_transform.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dlonng/Documents/pcl_work/BasicUsage/UsingAMatrixToTransformAPointCloud/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/matrix_transform.dir/matrix_transform.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matrix_transform.dir/matrix_transform.cpp.o -c /home/dlonng/Documents/pcl_work/BasicUsage/UsingAMatrixToTransformAPointCloud/matrix_transform.cpp

CMakeFiles/matrix_transform.dir/matrix_transform.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matrix_transform.dir/matrix_transform.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dlonng/Documents/pcl_work/BasicUsage/UsingAMatrixToTransformAPointCloud/matrix_transform.cpp > CMakeFiles/matrix_transform.dir/matrix_transform.cpp.i

CMakeFiles/matrix_transform.dir/matrix_transform.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matrix_transform.dir/matrix_transform.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dlonng/Documents/pcl_work/BasicUsage/UsingAMatrixToTransformAPointCloud/matrix_transform.cpp -o CMakeFiles/matrix_transform.dir/matrix_transform.cpp.s

CMakeFiles/matrix_transform.dir/matrix_transform.cpp.o.requires:

.PHONY : CMakeFiles/matrix_transform.dir/matrix_transform.cpp.o.requires

CMakeFiles/matrix_transform.dir/matrix_transform.cpp.o.provides: CMakeFiles/matrix_transform.dir/matrix_transform.cpp.o.requires
	$(MAKE) -f CMakeFiles/matrix_transform.dir/build.make CMakeFiles/matrix_transform.dir/matrix_transform.cpp.o.provides.build
.PHONY : CMakeFiles/matrix_transform.dir/matrix_transform.cpp.o.provides

CMakeFiles/matrix_transform.dir/matrix_transform.cpp.o.provides.build: CMakeFiles/matrix_transform.dir/matrix_transform.cpp.o


# Object files for target matrix_transform
matrix_transform_OBJECTS = \
"CMakeFiles/matrix_transform.dir/matrix_transform.cpp.o"

# External object files for target matrix_transform
matrix_transform_EXTERNAL_OBJECTS =

matrix_transform: CMakeFiles/matrix_transform.dir/matrix_transform.cpp.o
matrix_transform: CMakeFiles/matrix_transform.dir/build.make
matrix_transform: /usr/lib/libpcl_outofcore.so
matrix_transform: /usr/lib/libpcl_apps.so
matrix_transform: /usr/lib/libpcl_people.so
matrix_transform: /usr/lib/x86_64-linux-gnu/libboost_system.so
matrix_transform: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
matrix_transform: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
matrix_transform: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
matrix_transform: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
matrix_transform: /usr/lib/x86_64-linux-gnu/libboost_regex.so
matrix_transform: /usr/lib/x86_64-linux-gnu/libqhull.so
matrix_transform: /usr/lib/libOpenNI.so
matrix_transform: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
matrix_transform: /usr/lib/libpcl_surface.so
matrix_transform: /usr/lib/libpcl_keypoints.so
matrix_transform: /usr/lib/libpcl_stereo.so
matrix_transform: /usr/lib/libpcl_recognition.so
matrix_transform: /usr/lib/libpcl_registration.so
matrix_transform: /usr/lib/libpcl_tracking.so
matrix_transform: /usr/lib/libpcl_segmentation.so
matrix_transform: /usr/lib/libpcl_features.so
matrix_transform: /usr/lib/libpcl_filters.so
matrix_transform: /usr/lib/libpcl_sample_consensus.so
matrix_transform: /usr/lib/libvtkGenericFiltering.so.5.10.1
matrix_transform: /usr/lib/libvtkGeovis.so.5.10.1
matrix_transform: /usr/lib/libvtkCharts.so.5.10.1
matrix_transform: /usr/lib/libvtkViews.so.5.10.1
matrix_transform: /usr/lib/libvtkInfovis.so.5.10.1
matrix_transform: /usr/lib/libvtkWidgets.so.5.10.1
matrix_transform: /usr/lib/libvtkVolumeRendering.so.5.10.1
matrix_transform: /usr/lib/libvtkHybrid.so.5.10.1
matrix_transform: /usr/lib/libvtkParallel.so.5.10.1
matrix_transform: /usr/lib/libvtkRendering.so.5.10.1
matrix_transform: /usr/lib/libvtkImaging.so.5.10.1
matrix_transform: /usr/lib/libvtkGraphics.so.5.10.1
matrix_transform: /usr/lib/libvtkIO.so.5.10.1
matrix_transform: /usr/lib/libvtkFiltering.so.5.10.1
matrix_transform: /usr/lib/libvtkCommon.so.5.10.1
matrix_transform: /usr/lib/libvtksys.so.5.10.1
matrix_transform: /usr/lib/libpcl_ml.so
matrix_transform: /usr/lib/libpcl_visualization.so
matrix_transform: /usr/lib/libpcl_search.so
matrix_transform: /usr/lib/libpcl_kdtree.so
matrix_transform: /usr/lib/libpcl_io.so
matrix_transform: /usr/lib/libpcl_octree.so
matrix_transform: /usr/lib/libpcl_common.so
matrix_transform: CMakeFiles/matrix_transform.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dlonng/Documents/pcl_work/BasicUsage/UsingAMatrixToTransformAPointCloud/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable matrix_transform"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/matrix_transform.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/matrix_transform.dir/build: matrix_transform

.PHONY : CMakeFiles/matrix_transform.dir/build

CMakeFiles/matrix_transform.dir/requires: CMakeFiles/matrix_transform.dir/matrix_transform.cpp.o.requires

.PHONY : CMakeFiles/matrix_transform.dir/requires

CMakeFiles/matrix_transform.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/matrix_transform.dir/cmake_clean.cmake
.PHONY : CMakeFiles/matrix_transform.dir/clean

CMakeFiles/matrix_transform.dir/depend:
	cd /home/dlonng/Documents/pcl_work/BasicUsage/UsingAMatrixToTransformAPointCloud/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dlonng/Documents/pcl_work/BasicUsage/UsingAMatrixToTransformAPointCloud /home/dlonng/Documents/pcl_work/BasicUsage/UsingAMatrixToTransformAPointCloud /home/dlonng/Documents/pcl_work/BasicUsage/UsingAMatrixToTransformAPointCloud/build /home/dlonng/Documents/pcl_work/BasicUsage/UsingAMatrixToTransformAPointCloud/build /home/dlonng/Documents/pcl_work/BasicUsage/UsingAMatrixToTransformAPointCloud/build/CMakeFiles/matrix_transform.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/matrix_transform.dir/depend

