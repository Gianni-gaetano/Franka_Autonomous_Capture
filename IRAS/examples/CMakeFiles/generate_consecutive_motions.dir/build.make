# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/iras0401/libfranka

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/iras0401/libfranka/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/generate_consecutive_motions.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/generate_consecutive_motions.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/generate_consecutive_motions.dir/flags.make

examples/CMakeFiles/generate_consecutive_motions.dir/generate_consecutive_motions.cpp.o: examples/CMakeFiles/generate_consecutive_motions.dir/flags.make
examples/CMakeFiles/generate_consecutive_motions.dir/generate_consecutive_motions.cpp.o: ../examples/generate_consecutive_motions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/iras0401/libfranka/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/generate_consecutive_motions.dir/generate_consecutive_motions.cpp.o"
	cd /home/iras0401/libfranka/build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/generate_consecutive_motions.dir/generate_consecutive_motions.cpp.o -c /home/iras0401/libfranka/examples/generate_consecutive_motions.cpp

examples/CMakeFiles/generate_consecutive_motions.dir/generate_consecutive_motions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/generate_consecutive_motions.dir/generate_consecutive_motions.cpp.i"
	cd /home/iras0401/libfranka/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/iras0401/libfranka/examples/generate_consecutive_motions.cpp > CMakeFiles/generate_consecutive_motions.dir/generate_consecutive_motions.cpp.i

examples/CMakeFiles/generate_consecutive_motions.dir/generate_consecutive_motions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/generate_consecutive_motions.dir/generate_consecutive_motions.cpp.s"
	cd /home/iras0401/libfranka/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/iras0401/libfranka/examples/generate_consecutive_motions.cpp -o CMakeFiles/generate_consecutive_motions.dir/generate_consecutive_motions.cpp.s

# Object files for target generate_consecutive_motions
generate_consecutive_motions_OBJECTS = \
"CMakeFiles/generate_consecutive_motions.dir/generate_consecutive_motions.cpp.o"

# External object files for target generate_consecutive_motions
generate_consecutive_motions_EXTERNAL_OBJECTS =

examples/generate_consecutive_motions: examples/CMakeFiles/generate_consecutive_motions.dir/generate_consecutive_motions.cpp.o
examples/generate_consecutive_motions: examples/CMakeFiles/generate_consecutive_motions.dir/build.make
examples/generate_consecutive_motions: examples/libexamples_common.a
examples/generate_consecutive_motions: libfranka.so.0.13.0
examples/generate_consecutive_motions: examples/CMakeFiles/generate_consecutive_motions.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/iras0401/libfranka/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable generate_consecutive_motions"
	cd /home/iras0401/libfranka/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/generate_consecutive_motions.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/generate_consecutive_motions.dir/build: examples/generate_consecutive_motions

.PHONY : examples/CMakeFiles/generate_consecutive_motions.dir/build

examples/CMakeFiles/generate_consecutive_motions.dir/clean:
	cd /home/iras0401/libfranka/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/generate_consecutive_motions.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/generate_consecutive_motions.dir/clean

examples/CMakeFiles/generate_consecutive_motions.dir/depend:
	cd /home/iras0401/libfranka/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/iras0401/libfranka /home/iras0401/libfranka/examples /home/iras0401/libfranka/build /home/iras0401/libfranka/build/examples /home/iras0401/libfranka/build/examples/CMakeFiles/generate_consecutive_motions.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/generate_consecutive_motions.dir/depend

