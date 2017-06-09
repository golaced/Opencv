# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.0

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
CMAKE_SOURCE_DIR = /home/pi/Downloads/aruco-2.0.16

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Downloads/aruco-2.0.16/build

# Include any dependencies generated for this target.
include utils/CMakeFiles/aruco_tracker.dir/depend.make

# Include the progress variables for this target.
include utils/CMakeFiles/aruco_tracker.dir/progress.make

# Include the compile flags for this target's objects.
include utils/CMakeFiles/aruco_tracker.dir/flags.make

utils/CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.o: utils/CMakeFiles/aruco_tracker.dir/flags.make
utils/CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.o: ../utils/aruco_tracker.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/Downloads/aruco-2.0.16/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object utils/CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.o"
	cd /home/pi/Downloads/aruco-2.0.16/build/utils && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.o -c /home/pi/Downloads/aruco-2.0.16/utils/aruco_tracker.cpp

utils/CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.i"
	cd /home/pi/Downloads/aruco-2.0.16/build/utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/pi/Downloads/aruco-2.0.16/utils/aruco_tracker.cpp > CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.i

utils/CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.s"
	cd /home/pi/Downloads/aruco-2.0.16/build/utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/pi/Downloads/aruco-2.0.16/utils/aruco_tracker.cpp -o CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.s

utils/CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.o.requires:
.PHONY : utils/CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.o.requires

utils/CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.o.provides: utils/CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.o.requires
	$(MAKE) -f utils/CMakeFiles/aruco_tracker.dir/build.make utils/CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.o.provides.build
.PHONY : utils/CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.o.provides

utils/CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.o.provides.build: utils/CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.o

# Object files for target aruco_tracker
aruco_tracker_OBJECTS = \
"CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.o"

# External object files for target aruco_tracker
aruco_tracker_EXTERNAL_OBJECTS =

utils/aruco_tracker: utils/CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.o
utils/aruco_tracker: utils/CMakeFiles/aruco_tracker.dir/build.make
utils/aruco_tracker: src/libaruco.so.2.0.15
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_videostab.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_video.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_ts.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_superres.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_stitching.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_photo.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_ocl.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_objdetect.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_ml.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_legacy.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_imgproc.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_highgui.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_gpu.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_flann.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_features2d.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_core.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_contrib.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_calib3d.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_photo.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_legacy.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_video.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_objdetect.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_ml.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_calib3d.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_features2d.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_highgui.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_imgproc.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_flann.so.2.4.9
utils/aruco_tracker: /usr/lib/arm-linux-gnueabihf/libopencv_core.so.2.4.9
utils/aruco_tracker: utils/CMakeFiles/aruco_tracker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable aruco_tracker"
	cd /home/pi/Downloads/aruco-2.0.16/build/utils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/aruco_tracker.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
utils/CMakeFiles/aruco_tracker.dir/build: utils/aruco_tracker
.PHONY : utils/CMakeFiles/aruco_tracker.dir/build

utils/CMakeFiles/aruco_tracker.dir/requires: utils/CMakeFiles/aruco_tracker.dir/aruco_tracker.cpp.o.requires
.PHONY : utils/CMakeFiles/aruco_tracker.dir/requires

utils/CMakeFiles/aruco_tracker.dir/clean:
	cd /home/pi/Downloads/aruco-2.0.16/build/utils && $(CMAKE_COMMAND) -P CMakeFiles/aruco_tracker.dir/cmake_clean.cmake
.PHONY : utils/CMakeFiles/aruco_tracker.dir/clean

utils/CMakeFiles/aruco_tracker.dir/depend:
	cd /home/pi/Downloads/aruco-2.0.16/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Downloads/aruco-2.0.16 /home/pi/Downloads/aruco-2.0.16/utils /home/pi/Downloads/aruco-2.0.16/build /home/pi/Downloads/aruco-2.0.16/build/utils /home/pi/Downloads/aruco-2.0.16/build/utils/CMakeFiles/aruco_tracker.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : utils/CMakeFiles/aruco_tracker.dir/depend

