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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lacatus/Escritorio/3D/PCL_Proyects/PASS_T

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lacatus/Escritorio/3D/PCL_Proyects/PASS_T/build

# Include any dependencies generated for this target.
include CMakeFiles/pass.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pass.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pass.dir/flags.make

CMakeFiles/pass.dir/pass.cpp.o: CMakeFiles/pass.dir/flags.make
CMakeFiles/pass.dir/pass.cpp.o: ../pass.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lacatus/Escritorio/3D/PCL_Proyects/PASS_T/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/pass.dir/pass.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/pass.dir/pass.cpp.o -c /home/lacatus/Escritorio/3D/PCL_Proyects/PASS_T/pass.cpp

CMakeFiles/pass.dir/pass.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pass.dir/pass.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lacatus/Escritorio/3D/PCL_Proyects/PASS_T/pass.cpp > CMakeFiles/pass.dir/pass.cpp.i

CMakeFiles/pass.dir/pass.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pass.dir/pass.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lacatus/Escritorio/3D/PCL_Proyects/PASS_T/pass.cpp -o CMakeFiles/pass.dir/pass.cpp.s

CMakeFiles/pass.dir/pass.cpp.o.requires:
.PHONY : CMakeFiles/pass.dir/pass.cpp.o.requires

CMakeFiles/pass.dir/pass.cpp.o.provides: CMakeFiles/pass.dir/pass.cpp.o.requires
	$(MAKE) -f CMakeFiles/pass.dir/build.make CMakeFiles/pass.dir/pass.cpp.o.provides.build
.PHONY : CMakeFiles/pass.dir/pass.cpp.o.provides

CMakeFiles/pass.dir/pass.cpp.o.provides.build: CMakeFiles/pass.dir/pass.cpp.o

# Object files for target pass
pass_OBJECTS = \
"CMakeFiles/pass.dir/pass.cpp.o"

# External object files for target pass
pass_EXTERNAL_OBJECTS =

pass: CMakeFiles/pass.dir/pass.cpp.o
pass: /usr/lib/libboost_system-mt.so
pass: /usr/lib/libboost_filesystem-mt.so
pass: /usr/lib/libboost_thread-mt.so
pass: /usr/lib/libboost_date_time-mt.so
pass: /usr/lib/libboost_iostreams-mt.so
pass: /usr/lib/libboost_mpi-mt.so
pass: /usr/lib/libboost_serialization-mt.so
pass: /usr/local/lib/libpcl_common.so
pass: /usr/local/lib/libpcl_octree.so
pass: /usr/lib/libOpenNI.so
pass: /usr/lib/libvtkCommon.so.5.8.0
pass: /usr/lib/libvtkRendering.so.5.8.0
pass: /usr/lib/libvtkHybrid.so.5.8.0
pass: /usr/lib/libvtkCharts.so.5.8.0
pass: /usr/local/lib/libpcl_io.so
pass: /usr/lib/libflann_cpp_s.a
pass: /usr/local/lib/libpcl_kdtree.so
pass: /usr/local/lib/libpcl_search.so
pass: /usr/local/lib/libpcl_sample_consensus.so
pass: /usr/local/lib/libpcl_filters.so
pass: /usr/local/lib/libpcl_features.so
pass: /usr/local/lib/libpcl_segmentation.so
pass: /usr/local/lib/libpcl_visualization.so
pass: /usr/lib/libqhull.so
pass: /usr/local/lib/libpcl_surface.so
pass: /usr/local/lib/libpcl_registration.so
pass: /usr/local/lib/libpcl_keypoints.so
pass: /usr/local/lib/libpcl_tracking.so
pass: /usr/local/lib/libpcl_recognition.so
pass: /usr/local/lib/libpcl_outofcore.so
pass: /usr/local/lib/libpcl_people.so
pass: /usr/lib/libboost_system-mt.so
pass: /usr/lib/libboost_filesystem-mt.so
pass: /usr/lib/libboost_thread-mt.so
pass: /usr/lib/libboost_date_time-mt.so
pass: /usr/lib/libboost_iostreams-mt.so
pass: /usr/lib/libboost_mpi-mt.so
pass: /usr/lib/libboost_serialization-mt.so
pass: /usr/lib/libqhull.so
pass: /usr/lib/libOpenNI.so
pass: /usr/lib/libflann_cpp_s.a
pass: /usr/lib/libvtkCommon.so.5.8.0
pass: /usr/lib/libvtkRendering.so.5.8.0
pass: /usr/lib/libvtkHybrid.so.5.8.0
pass: /usr/lib/libvtkCharts.so.5.8.0
pass: /usr/local/lib/libopencv_calib3d.so
pass: /usr/local/lib/libopencv_contrib.so
pass: /usr/local/lib/libopencv_core.so
pass: /usr/local/lib/libopencv_features2d.so
pass: /usr/local/lib/libopencv_flann.so
pass: /usr/local/lib/libopencv_gpu.so
pass: /usr/local/lib/libopencv_highgui.so
pass: /usr/local/lib/libopencv_imgproc.so
pass: /usr/local/lib/libopencv_legacy.so
pass: /usr/local/lib/libopencv_ml.so
pass: /usr/local/lib/libopencv_nonfree.so
pass: /usr/local/lib/libopencv_objdetect.so
pass: /usr/local/lib/libopencv_photo.so
pass: /usr/local/lib/libopencv_stitching.so
pass: /usr/local/lib/libopencv_ts.so
pass: /usr/local/lib/libopencv_video.so
pass: /usr/local/lib/libopencv_videostab.so
pass: /usr/local/lib/libpcl_common.so
pass: /usr/local/lib/libpcl_octree.so
pass: /usr/local/lib/libpcl_io.so
pass: /usr/local/lib/libpcl_kdtree.so
pass: /usr/local/lib/libpcl_search.so
pass: /usr/local/lib/libpcl_sample_consensus.so
pass: /usr/local/lib/libpcl_filters.so
pass: /usr/local/lib/libpcl_features.so
pass: /usr/local/lib/libpcl_segmentation.so
pass: /usr/local/lib/libpcl_visualization.so
pass: /usr/local/lib/libpcl_surface.so
pass: /usr/local/lib/libpcl_registration.so
pass: /usr/local/lib/libpcl_keypoints.so
pass: /usr/local/lib/libpcl_tracking.so
pass: /usr/local/lib/libpcl_recognition.so
pass: /usr/local/lib/libpcl_outofcore.so
pass: /usr/local/lib/libpcl_people.so
pass: /usr/local/lib/libopencv_calib3d.so
pass: /usr/local/lib/libopencv_contrib.so
pass: /usr/local/lib/libopencv_core.so
pass: /usr/local/lib/libopencv_features2d.so
pass: /usr/local/lib/libopencv_flann.so
pass: /usr/local/lib/libopencv_gpu.so
pass: /usr/local/lib/libopencv_highgui.so
pass: /usr/local/lib/libopencv_imgproc.so
pass: /usr/local/lib/libopencv_legacy.so
pass: /usr/local/lib/libopencv_ml.so
pass: /usr/local/lib/libopencv_nonfree.so
pass: /usr/local/lib/libopencv_objdetect.so
pass: /usr/local/lib/libopencv_photo.so
pass: /usr/local/lib/libopencv_stitching.so
pass: /usr/local/lib/libopencv_ts.so
pass: /usr/local/lib/libopencv_video.so
pass: /usr/local/lib/libopencv_videostab.so
pass: /usr/lib/libvtkViews.so.5.8.0
pass: /usr/lib/libvtkInfovis.so.5.8.0
pass: /usr/lib/libvtkWidgets.so.5.8.0
pass: /usr/lib/libvtkHybrid.so.5.8.0
pass: /usr/lib/libvtkParallel.so.5.8.0
pass: /usr/lib/libvtkVolumeRendering.so.5.8.0
pass: /usr/lib/libvtkRendering.so.5.8.0
pass: /usr/lib/libvtkGraphics.so.5.8.0
pass: /usr/lib/libvtkImaging.so.5.8.0
pass: /usr/lib/libvtkIO.so.5.8.0
pass: /usr/lib/libvtkFiltering.so.5.8.0
pass: /usr/lib/libvtkCommon.so.5.8.0
pass: /usr/lib/libvtksys.so.5.8.0
pass: CMakeFiles/pass.dir/build.make
pass: CMakeFiles/pass.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable pass"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pass.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pass.dir/build: pass
.PHONY : CMakeFiles/pass.dir/build

CMakeFiles/pass.dir/requires: CMakeFiles/pass.dir/pass.cpp.o.requires
.PHONY : CMakeFiles/pass.dir/requires

CMakeFiles/pass.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pass.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pass.dir/clean

CMakeFiles/pass.dir/depend:
	cd /home/lacatus/Escritorio/3D/PCL_Proyects/PASS_T/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lacatus/Escritorio/3D/PCL_Proyects/PASS_T /home/lacatus/Escritorio/3D/PCL_Proyects/PASS_T /home/lacatus/Escritorio/3D/PCL_Proyects/PASS_T/build /home/lacatus/Escritorio/3D/PCL_Proyects/PASS_T/build /home/lacatus/Escritorio/3D/PCL_Proyects/PASS_T/build/CMakeFiles/pass.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pass.dir/depend

