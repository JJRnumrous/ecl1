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
include validation/tests/CMakeFiles/ecl_tests_data_validator.dir/depend.make

# Include the progress variables for this target.
include validation/tests/CMakeFiles/ecl_tests_data_validator.dir/progress.make

# Include the compile flags for this target's objects.
include validation/tests/CMakeFiles/ecl_tests_data_validator.dir/flags.make

validation/tests/CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.o: validation/tests/CMakeFiles/ecl_tests_data_validator.dir/flags.make
validation/tests/CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.o: ../validation/tests/test_data_validator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jjr/DataFiles/Development/ecl1/Build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object validation/tests/CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.o"
	cd /home/jjr/DataFiles/Development/ecl1/Build/validation/tests && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.o -c /home/jjr/DataFiles/Development/ecl1/validation/tests/test_data_validator.cpp

validation/tests/CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.i"
	cd /home/jjr/DataFiles/Development/ecl1/Build/validation/tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jjr/DataFiles/Development/ecl1/validation/tests/test_data_validator.cpp > CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.i

validation/tests/CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.s"
	cd /home/jjr/DataFiles/Development/ecl1/Build/validation/tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jjr/DataFiles/Development/ecl1/validation/tests/test_data_validator.cpp -o CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.s

validation/tests/CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.o.requires:

.PHONY : validation/tests/CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.o.requires

validation/tests/CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.o.provides: validation/tests/CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.o.requires
	$(MAKE) -f validation/tests/CMakeFiles/ecl_tests_data_validator.dir/build.make validation/tests/CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.o.provides.build
.PHONY : validation/tests/CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.o.provides

validation/tests/CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.o.provides.build: validation/tests/CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.o


# Object files for target ecl_tests_data_validator
ecl_tests_data_validator_OBJECTS = \
"CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.o"

# External object files for target ecl_tests_data_validator
ecl_tests_data_validator_EXTERNAL_OBJECTS =

validation/tests/ecl_tests_data_validator: validation/tests/CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.o
validation/tests/ecl_tests_data_validator: validation/tests/CMakeFiles/ecl_tests_data_validator.dir/build.make
validation/tests/ecl_tests_data_validator: validation/libecl_validation.a
validation/tests/ecl_tests_data_validator: validation/tests/CMakeFiles/ecl_tests_data_validator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jjr/DataFiles/Development/ecl1/Build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ecl_tests_data_validator"
	cd /home/jjr/DataFiles/Development/ecl1/Build/validation/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ecl_tests_data_validator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
validation/tests/CMakeFiles/ecl_tests_data_validator.dir/build: validation/tests/ecl_tests_data_validator

.PHONY : validation/tests/CMakeFiles/ecl_tests_data_validator.dir/build

validation/tests/CMakeFiles/ecl_tests_data_validator.dir/requires: validation/tests/CMakeFiles/ecl_tests_data_validator.dir/test_data_validator.cpp.o.requires

.PHONY : validation/tests/CMakeFiles/ecl_tests_data_validator.dir/requires

validation/tests/CMakeFiles/ecl_tests_data_validator.dir/clean:
	cd /home/jjr/DataFiles/Development/ecl1/Build/validation/tests && $(CMAKE_COMMAND) -P CMakeFiles/ecl_tests_data_validator.dir/cmake_clean.cmake
.PHONY : validation/tests/CMakeFiles/ecl_tests_data_validator.dir/clean

validation/tests/CMakeFiles/ecl_tests_data_validator.dir/depend:
	cd /home/jjr/DataFiles/Development/ecl1/Build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jjr/DataFiles/Development/ecl1 /home/jjr/DataFiles/Development/ecl1/validation/tests /home/jjr/DataFiles/Development/ecl1/Build /home/jjr/DataFiles/Development/ecl1/Build/validation/tests /home/jjr/DataFiles/Development/ecl1/Build/validation/tests/CMakeFiles/ecl_tests_data_validator.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : validation/tests/CMakeFiles/ecl_tests_data_validator.dir/depend

