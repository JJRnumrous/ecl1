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
include geo_lookup/CMakeFiles/ecl_geo_lookup.dir/depend.make

# Include the progress variables for this target.
include geo_lookup/CMakeFiles/ecl_geo_lookup.dir/progress.make

# Include the compile flags for this target's objects.
include geo_lookup/CMakeFiles/ecl_geo_lookup.dir/flags.make

geo_lookup/CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.o: geo_lookup/CMakeFiles/ecl_geo_lookup.dir/flags.make
geo_lookup/CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.o: ../geo_lookup/geo_mag_declination.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jjr/DataFiles/Development/ecl1/Build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object geo_lookup/CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.o"
	cd /home/jjr/DataFiles/Development/ecl1/Build/geo_lookup && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.o -c /home/jjr/DataFiles/Development/ecl1/geo_lookup/geo_mag_declination.cpp

geo_lookup/CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.i"
	cd /home/jjr/DataFiles/Development/ecl1/Build/geo_lookup && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jjr/DataFiles/Development/ecl1/geo_lookup/geo_mag_declination.cpp > CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.i

geo_lookup/CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.s"
	cd /home/jjr/DataFiles/Development/ecl1/Build/geo_lookup && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jjr/DataFiles/Development/ecl1/geo_lookup/geo_mag_declination.cpp -o CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.s

geo_lookup/CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.o.requires:

.PHONY : geo_lookup/CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.o.requires

geo_lookup/CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.o.provides: geo_lookup/CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.o.requires
	$(MAKE) -f geo_lookup/CMakeFiles/ecl_geo_lookup.dir/build.make geo_lookup/CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.o.provides.build
.PHONY : geo_lookup/CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.o.provides

geo_lookup/CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.o.provides.build: geo_lookup/CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.o


# Object files for target ecl_geo_lookup
ecl_geo_lookup_OBJECTS = \
"CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.o"

# External object files for target ecl_geo_lookup
ecl_geo_lookup_EXTERNAL_OBJECTS =

geo_lookup/libecl_geo_lookup.a: geo_lookup/CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.o
geo_lookup/libecl_geo_lookup.a: geo_lookup/CMakeFiles/ecl_geo_lookup.dir/build.make
geo_lookup/libecl_geo_lookup.a: geo_lookup/CMakeFiles/ecl_geo_lookup.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jjr/DataFiles/Development/ecl1/Build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libecl_geo_lookup.a"
	cd /home/jjr/DataFiles/Development/ecl1/Build/geo_lookup && $(CMAKE_COMMAND) -P CMakeFiles/ecl_geo_lookup.dir/cmake_clean_target.cmake
	cd /home/jjr/DataFiles/Development/ecl1/Build/geo_lookup && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ecl_geo_lookup.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
geo_lookup/CMakeFiles/ecl_geo_lookup.dir/build: geo_lookup/libecl_geo_lookup.a

.PHONY : geo_lookup/CMakeFiles/ecl_geo_lookup.dir/build

geo_lookup/CMakeFiles/ecl_geo_lookup.dir/requires: geo_lookup/CMakeFiles/ecl_geo_lookup.dir/geo_mag_declination.cpp.o.requires

.PHONY : geo_lookup/CMakeFiles/ecl_geo_lookup.dir/requires

geo_lookup/CMakeFiles/ecl_geo_lookup.dir/clean:
	cd /home/jjr/DataFiles/Development/ecl1/Build/geo_lookup && $(CMAKE_COMMAND) -P CMakeFiles/ecl_geo_lookup.dir/cmake_clean.cmake
.PHONY : geo_lookup/CMakeFiles/ecl_geo_lookup.dir/clean

geo_lookup/CMakeFiles/ecl_geo_lookup.dir/depend:
	cd /home/jjr/DataFiles/Development/ecl1/Build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jjr/DataFiles/Development/ecl1 /home/jjr/DataFiles/Development/ecl1/geo_lookup /home/jjr/DataFiles/Development/ecl1/Build /home/jjr/DataFiles/Development/ecl1/Build/geo_lookup /home/jjr/DataFiles/Development/ecl1/Build/geo_lookup/CMakeFiles/ecl_geo_lookup.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : geo_lookup/CMakeFiles/ecl_geo_lookup.dir/depend
