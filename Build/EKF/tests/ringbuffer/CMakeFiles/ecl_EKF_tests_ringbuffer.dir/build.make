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
CMAKE_SOURCE_DIR = /home/jjr/DataFiles/Development/ecl1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jjr/DataFiles/Development/ecl1/Build

# Include any dependencies generated for this target.
include EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/depend.make

# Include the progress variables for this target.
include EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/progress.make

# Include the compile flags for this target's objects.
include EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/flags.make

EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.o: EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/flags.make
EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.o: ../EKF/tests/ringbuffer/ringbuffer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jjr/DataFiles/Development/ecl1/Build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.o"
	cd /home/jjr/DataFiles/Development/ecl1/Build/EKF/tests/ringbuffer && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.o -c /home/jjr/DataFiles/Development/ecl1/EKF/tests/ringbuffer/ringbuffer.cpp

EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.i"
	cd /home/jjr/DataFiles/Development/ecl1/Build/EKF/tests/ringbuffer && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jjr/DataFiles/Development/ecl1/EKF/tests/ringbuffer/ringbuffer.cpp > CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.i

EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.s"
	cd /home/jjr/DataFiles/Development/ecl1/Build/EKF/tests/ringbuffer && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jjr/DataFiles/Development/ecl1/EKF/tests/ringbuffer/ringbuffer.cpp -o CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.s

EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.o.requires:

.PHONY : EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.o.requires

EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.o.provides: EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.o.requires
	$(MAKE) -f EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/build.make EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.o.provides.build
.PHONY : EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.o.provides

EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.o.provides.build: EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.o


# Object files for target ecl_EKF_tests_ringbuffer
ecl_EKF_tests_ringbuffer_OBJECTS = \
"CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.o"

# External object files for target ecl_EKF_tests_ringbuffer
ecl_EKF_tests_ringbuffer_EXTERNAL_OBJECTS =

EKF/tests/ringbuffer/ecl_EKF_tests_ringbuffer: EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.o
EKF/tests/ringbuffer/ecl_EKF_tests_ringbuffer: EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/build.make
EKF/tests/ringbuffer/ecl_EKF_tests_ringbuffer: EKF/libecl_EKF.a
EKF/tests/ringbuffer/ecl_EKF_tests_ringbuffer: geo/libecl_geo.a
EKF/tests/ringbuffer/ecl_EKF_tests_ringbuffer: geo_lookup/libecl_geo_lookup.a
EKF/tests/ringbuffer/ecl_EKF_tests_ringbuffer: mathlib/libmathlib.a
EKF/tests/ringbuffer/ecl_EKF_tests_ringbuffer: EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jjr/DataFiles/Development/ecl1/Build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ecl_EKF_tests_ringbuffer"
	cd /home/jjr/DataFiles/Development/ecl1/Build/EKF/tests/ringbuffer && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ecl_EKF_tests_ringbuffer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/build: EKF/tests/ringbuffer/ecl_EKF_tests_ringbuffer

.PHONY : EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/build

EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/requires: EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/ringbuffer.cpp.o.requires

.PHONY : EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/requires

EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/clean:
	cd /home/jjr/DataFiles/Development/ecl1/Build/EKF/tests/ringbuffer && $(CMAKE_COMMAND) -P CMakeFiles/ecl_EKF_tests_ringbuffer.dir/cmake_clean.cmake
.PHONY : EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/clean

EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/depend:
	cd /home/jjr/DataFiles/Development/ecl1/Build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jjr/DataFiles/Development/ecl1 /home/jjr/DataFiles/Development/ecl1/EKF/tests/ringbuffer /home/jjr/DataFiles/Development/ecl1/Build /home/jjr/DataFiles/Development/ecl1/Build/EKF/tests/ringbuffer /home/jjr/DataFiles/Development/ecl1/Build/EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : EKF/tests/ringbuffer/CMakeFiles/ecl_EKF_tests_ringbuffer.dir/depend

