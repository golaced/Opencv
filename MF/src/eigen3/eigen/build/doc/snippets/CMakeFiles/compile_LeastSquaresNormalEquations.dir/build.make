# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_SOURCE_DIR = /home/pi/QT/MF/src/eigen3/eigen

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/QT/MF/src/eigen3/eigen/build

# Include any dependencies generated for this target.
include doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/depend.make

# Include the progress variables for this target.
include doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/progress.make

# Include the compile flags for this target's objects.
include doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/flags.make

doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.o: doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/flags.make
doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.o: doc/snippets/compile_LeastSquaresNormalEquations.cpp
doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.o: ../doc/snippets/LeastSquaresNormalEquations.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/QT/MF/src/eigen3/eigen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.o"
	cd /home/pi/QT/MF/src/eigen3/eigen/build/doc/snippets && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.o -c /home/pi/QT/MF/src/eigen3/eigen/build/doc/snippets/compile_LeastSquaresNormalEquations.cpp

doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.i"
	cd /home/pi/QT/MF/src/eigen3/eigen/build/doc/snippets && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/QT/MF/src/eigen3/eigen/build/doc/snippets/compile_LeastSquaresNormalEquations.cpp > CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.i

doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.s"
	cd /home/pi/QT/MF/src/eigen3/eigen/build/doc/snippets && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/QT/MF/src/eigen3/eigen/build/doc/snippets/compile_LeastSquaresNormalEquations.cpp -o CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.s

doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.o.requires:

.PHONY : doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.o.requires

doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.o.provides: doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.o.requires
	$(MAKE) -f doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/build.make doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.o.provides.build
.PHONY : doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.o.provides

doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.o.provides.build: doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.o


# Object files for target compile_LeastSquaresNormalEquations
compile_LeastSquaresNormalEquations_OBJECTS = \
"CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.o"

# External object files for target compile_LeastSquaresNormalEquations
compile_LeastSquaresNormalEquations_EXTERNAL_OBJECTS =

doc/snippets/compile_LeastSquaresNormalEquations: doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.o
doc/snippets/compile_LeastSquaresNormalEquations: doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/build.make
doc/snippets/compile_LeastSquaresNormalEquations: doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/QT/MF/src/eigen3/eigen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable compile_LeastSquaresNormalEquations"
	cd /home/pi/QT/MF/src/eigen3/eigen/build/doc/snippets && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/compile_LeastSquaresNormalEquations.dir/link.txt --verbose=$(VERBOSE)
	cd /home/pi/QT/MF/src/eigen3/eigen/build/doc/snippets && ./compile_LeastSquaresNormalEquations >/home/pi/QT/MF/src/eigen3/eigen/build/doc/snippets/LeastSquaresNormalEquations.out

# Rule to build all files generated by this target.
doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/build: doc/snippets/compile_LeastSquaresNormalEquations

.PHONY : doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/build

doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/requires: doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/compile_LeastSquaresNormalEquations.cpp.o.requires

.PHONY : doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/requires

doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/clean:
	cd /home/pi/QT/MF/src/eigen3/eigen/build/doc/snippets && $(CMAKE_COMMAND) -P CMakeFiles/compile_LeastSquaresNormalEquations.dir/cmake_clean.cmake
.PHONY : doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/clean

doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/depend:
	cd /home/pi/QT/MF/src/eigen3/eigen/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/QT/MF/src/eigen3/eigen /home/pi/QT/MF/src/eigen3/eigen/doc/snippets /home/pi/QT/MF/src/eigen3/eigen/build /home/pi/QT/MF/src/eigen3/eigen/build/doc/snippets /home/pi/QT/MF/src/eigen3/eigen/build/doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : doc/snippets/CMakeFiles/compile_LeastSquaresNormalEquations.dir/depend

