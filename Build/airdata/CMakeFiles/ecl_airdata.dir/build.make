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
include airdata/CMakeFiles/ecl_airdata.dir/depend.make

# Include the progress variables for this target.
include airdata/CMakeFiles/ecl_airdata.dir/progress.make

# Include the compile flags for this target's objects.
include airdata/CMakeFiles/ecl_airdata.dir/flags.make

airdata/CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.o: airdata/CMakeFiles/ecl_airdata.dir/flags.make
airdata/CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.o: ../airdata/WindEstimator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jjr/DataFiles/Development/ecl1/Build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object airdata/CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.o"
	cd /home/jjr/DataFiles/Development/ecl1/Build/airdata && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.o -c /home/jjr/DataFiles/Development/ecl1/airdata/WindEstimator.cpp

airdata/CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.i"
	cd /home/jjr/DataFiles/Development/ecl1/Build/airdata && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jjr/DataFiles/Development/ecl1/airdata/WindEstimator.cpp > CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.i

airdata/CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.s"
	cd /home/jjr/DataFiles/Development/ecl1/Build/airdata && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jjr/DataFiles/Development/ecl1/airdata/WindEstimator.cpp -o CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.s

airdata/CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.o.requires:

.PHONY : airdata/CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.o.requires

airdata/CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.o.provides: airdata/CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.o.requires
	$(MAKE) -f airdata/CMakeFiles/ecl_airdata.dir/build.make airdata/CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.o.provides.build
.PHONY : airdata/CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.o.provides

airdata/CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.o.provides.build: airdata/CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.o


# Object files for target ecl_airdata
ecl_airdata_OBJECTS = \
"CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.o"

# External object files for target ecl_airdata
ecl_airdata_EXTERNAL_OBJECTS =

airdata/libecl_airdata.a: airdata/CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.o
airdata/libecl_airdata.a: airdata/CMakeFiles/ecl_airdata.dir/build.make
airdata/libecl_airdata.a: airdata/CMakeFiles/ecl_airdata.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jjr/DataFiles/Development/ecl1/Build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libecl_airdata.a"
	cd /home/jjr/DataFiles/Development/ecl1/Build/airdata && $(CMAKE_COMMAND) -P CMakeFiles/ecl_airdata.dir/cmake_clean_target.cmake
	cd /home/jjr/DataFiles/Development/ecl1/Build/airdata && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ecl_airdata.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
airdata/CMakeFiles/ecl_airdata.dir/build: airdata/libecl_airdata.a

.PHONY : airdata/CMakeFiles/ecl_airdata.dir/build

airdata/CMakeFiles/ecl_airdata.dir/requires: airdata/CMakeFiles/ecl_airdata.dir/WindEstimator.cpp.o.requires

.PHONY : airdata/CMakeFiles/ecl_airdata.dir/requires

airdata/CMakeFiles/ecl_airdata.dir/clean:
	cd /home/jjr/DataFiles/Development/ecl1/Build/airdata && $(CMAKE_COMMAND) -P CMakeFiles/ecl_airdata.dir/cmake_clean.cmake
.PHONY : airdata/CMakeFiles/ecl_airdata.dir/clean

airdata/CMakeFiles/ecl_airdata.dir/depend:
	cd /home/jjr/DataFiles/Development/ecl1/Build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jjr/DataFiles/Development/ecl1 /home/jjr/DataFiles/Development/ecl1/airdata /home/jjr/DataFiles/Development/ecl1/Build /home/jjr/DataFiles/Development/ecl1/Build/airdata /home/jjr/DataFiles/Development/ecl1/Build/airdata/CMakeFiles/ecl_airdata.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : airdata/CMakeFiles/ecl_airdata.dir/depend
