# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_COMMAND = /home/leo/Downloads/clion-2017.1.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/leo/Downloads/clion-2017.1.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/leo/Desktop/Untitled Folder"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/leo/Desktop/Untitled Folder/cmake-build-debug"

# Include any dependencies generated for this target.
include CMakeFiles/SematicSLAM.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SematicSLAM.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SematicSLAM.dir/flags.make

CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.o: CMakeFiles/SematicSLAM.dir/flags.make
CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.o: ../src/SematicSLAM.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/leo/Desktop/Untitled Folder/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.o -c "/home/leo/Desktop/Untitled Folder/src/SematicSLAM.cpp"

CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/leo/Desktop/Untitled Folder/src/SematicSLAM.cpp" > CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.i

CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/leo/Desktop/Untitled Folder/src/SematicSLAM.cpp" -o CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.s

CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.o.requires:

.PHONY : CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.o.requires

CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.o.provides: CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.o.requires
	$(MAKE) -f CMakeFiles/SematicSLAM.dir/build.make CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.o.provides.build
.PHONY : CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.o.provides

CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.o.provides.build: CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.o


# Object files for target SematicSLAM
SematicSLAM_OBJECTS = \
"CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.o"

# External object files for target SematicSLAM
SematicSLAM_EXTERNAL_OBJECTS =

SematicSLAM: CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.o
SematicSLAM: CMakeFiles/SematicSLAM.dir/build.make
SematicSLAM: libSEMATIC_SLAM.so
SematicSLAM: /usr/local/lib/libopencv_cudabgsegm.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_cudaobjdetect.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_cudastereo.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_dnn.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_ml.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_shape.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_stitching.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_cudafeatures2d.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_superres.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_cudacodec.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_videostab.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_cudaoptflow.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_cudalegacy.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_calib3d.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_cudawarping.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_features2d.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_flann.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_highgui.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_objdetect.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_photo.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_cudaimgproc.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_cudafilters.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_cudaarithm.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_video.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_videoio.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_imgcodecs.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_imgproc.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_viz.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_core.so.3.3.0
SematicSLAM: /usr/local/lib/libopencv_cudev.so.3.3.0
SematicSLAM: /home/leo/Downloads/Pangolin/build/src/libpangolin.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libGLU.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libGL.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libSM.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libICE.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libX11.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libXext.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libGLEW.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libSM.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libICE.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libX11.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libXext.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libGLEW.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libdc1394.so
SematicSLAM: /usr/local/lib/x86_64-linux-gnu/librealsense.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libpng.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libz.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libjpeg.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libtiff.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libIlmImf.so
SematicSLAM: ../ORB_SLAM2/Thirdparty/DBoW2/lib/libDBoW2.so
SematicSLAM: ../ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
SematicSLAM: /opt/ros/indigo/lib/libpcl_ros_filters.so
SematicSLAM: /opt/ros/indigo/lib/libpcl_ros_io.so
SematicSLAM: /opt/ros/indigo/lib/libpcl_ros_tf.so
SematicSLAM: /usr/lib/libpcl_common.so
SematicSLAM: /usr/lib/libpcl_octree.so
SematicSLAM: /usr/lib/libpcl_io.so
SematicSLAM: /usr/lib/libpcl_kdtree.so
SematicSLAM: /usr/lib/libpcl_search.so
SematicSLAM: /usr/lib/libpcl_sample_consensus.so
SematicSLAM: /usr/lib/libpcl_filters.so
SematicSLAM: /usr/lib/libpcl_features.so
SematicSLAM: /usr/lib/libpcl_keypoints.so
SematicSLAM: /usr/lib/libpcl_segmentation.so
SematicSLAM: /usr/lib/libpcl_visualization.so
SematicSLAM: /usr/lib/libpcl_outofcore.so
SematicSLAM: /usr/lib/libpcl_registration.so
SematicSLAM: /usr/lib/libpcl_recognition.so
SematicSLAM: /usr/lib/libpcl_surface.so
SematicSLAM: /usr/lib/libpcl_people.so
SematicSLAM: /usr/lib/libpcl_tracking.so
SematicSLAM: /usr/lib/libpcl_apps.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libqhull.so
SematicSLAM: /usr/lib/libOpenNI.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
SematicSLAM: /usr/lib/libvtkCommon.so.5.8.0
SematicSLAM: /usr/lib/libvtkRendering.so.5.8.0
SematicSLAM: /usr/lib/libvtkHybrid.so.5.8.0
SematicSLAM: /usr/lib/libvtkCharts.so.5.8.0
SematicSLAM: /opt/ros/indigo/lib/libdynamic_reconfigure_config_init_mutex.so
SematicSLAM: /opt/ros/indigo/lib/libnodeletlib.so
SematicSLAM: /opt/ros/indigo/lib/libbondcpp.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libuuid.so
SematicSLAM: /opt/ros/indigo/lib/librosbag.so
SematicSLAM: /opt/ros/indigo/lib/librosbag_storage.so
SematicSLAM: /opt/ros/indigo/lib/libroslz4.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/liblz4.so
SematicSLAM: /opt/ros/indigo/lib/libtopic_tools.so
SematicSLAM: /opt/ros/indigo/lib/libtf.so
SematicSLAM: /opt/ros/indigo/lib/libtf2_ros.so
SematicSLAM: /opt/ros/indigo/lib/libactionlib.so
SematicSLAM: /opt/ros/indigo/lib/libtf2.so
SematicSLAM: /opt/ros/indigo/lib/libimage_transport.so
SematicSLAM: /opt/ros/indigo/lib/libmessage_filters.so
SematicSLAM: /opt/ros/indigo/lib/libclass_loader.so
SematicSLAM: /usr/lib/libPocoFoundation.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libdl.so
SematicSLAM: /opt/ros/indigo/lib/libroscpp.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libboost_signals.so
SematicSLAM: /opt/ros/indigo/lib/librosconsole.so
SematicSLAM: /opt/ros/indigo/lib/librosconsole_log4cxx.so
SematicSLAM: /opt/ros/indigo/lib/librosconsole_backend_interface.so
SematicSLAM: /usr/lib/liblog4cxx.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libboost_regex.so
SematicSLAM: /opt/ros/indigo/lib/libxmlrpcpp.so
SematicSLAM: /opt/ros/indigo/lib/libroslib.so
SematicSLAM: /opt/ros/indigo/lib/librospack.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libpython2.7.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libtinyxml.so
SematicSLAM: /opt/ros/indigo/lib/libroscpp_serialization.so
SematicSLAM: /opt/ros/indigo/lib/librostime.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
SematicSLAM: /opt/ros/indigo/lib/libcpp_common.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libboost_system.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libboost_thread.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libpthread.so
SematicSLAM: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
SematicSLAM: /opt/ros/indigo/lib/libcv_bridge.so
SematicSLAM: ../YOLO_V3/lib/libdarknet.so
SematicSLAM: CMakeFiles/SematicSLAM.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/leo/Desktop/Untitled Folder/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable SematicSLAM"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SematicSLAM.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SematicSLAM.dir/build: SematicSLAM

.PHONY : CMakeFiles/SematicSLAM.dir/build

CMakeFiles/SematicSLAM.dir/requires: CMakeFiles/SematicSLAM.dir/src/SematicSLAM.cpp.o.requires

.PHONY : CMakeFiles/SematicSLAM.dir/requires

CMakeFiles/SematicSLAM.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SematicSLAM.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SematicSLAM.dir/clean

CMakeFiles/SematicSLAM.dir/depend:
	cd "/home/leo/Desktop/Untitled Folder/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/leo/Desktop/Untitled Folder" "/home/leo/Desktop/Untitled Folder" "/home/leo/Desktop/Untitled Folder/cmake-build-debug" "/home/leo/Desktop/Untitled Folder/cmake-build-debug" "/home/leo/Desktop/Untitled Folder/cmake-build-debug/CMakeFiles/SematicSLAM.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/SematicSLAM.dir/depend

