# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_SOURCE_DIR = /home/haochengli/KiteEnergySystems

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/haochengli/KiteEnergySystems/build

# Include any dependencies generated for this target.
include kite_dynamics/CMakeFiles/kite_utils.dir/depend.make

# Include the progress variables for this target.
include kite_dynamics/CMakeFiles/kite_utils.dir/progress.make

# Include the compile flags for this target's objects.
include kite_dynamics/CMakeFiles/kite_utils.dir/flags.make

kite_dynamics/CMakeFiles/kite_utils.dir/src/utils/KiteState.cc.o: kite_dynamics/CMakeFiles/kite_utils.dir/flags.make
kite_dynamics/CMakeFiles/kite_utils.dir/src/utils/KiteState.cc.o: ../kite_dynamics/src/utils/KiteState.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/haochengli/KiteEnergySystems/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object kite_dynamics/CMakeFiles/kite_utils.dir/src/utils/KiteState.cc.o"
	cd /home/haochengli/KiteEnergySystems/build/kite_dynamics && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kite_utils.dir/src/utils/KiteState.cc.o -c /home/haochengli/KiteEnergySystems/kite_dynamics/src/utils/KiteState.cc

kite_dynamics/CMakeFiles/kite_utils.dir/src/utils/KiteState.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kite_utils.dir/src/utils/KiteState.cc.i"
	cd /home/haochengli/KiteEnergySystems/build/kite_dynamics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/haochengli/KiteEnergySystems/kite_dynamics/src/utils/KiteState.cc > CMakeFiles/kite_utils.dir/src/utils/KiteState.cc.i

kite_dynamics/CMakeFiles/kite_utils.dir/src/utils/KiteState.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kite_utils.dir/src/utils/KiteState.cc.s"
	cd /home/haochengli/KiteEnergySystems/build/kite_dynamics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/haochengli/KiteEnergySystems/kite_dynamics/src/utils/KiteState.cc -o CMakeFiles/kite_utils.dir/src/utils/KiteState.cc.s

# Object files for target kite_utils
kite_utils_OBJECTS = \
"CMakeFiles/kite_utils.dir/src/utils/KiteState.cc.o"

# External object files for target kite_utils
kite_utils_EXTERNAL_OBJECTS =

kite_dynamics/libkite_utils.so: kite_dynamics/CMakeFiles/kite_utils.dir/src/utils/KiteState.cc.o
kite_dynamics/libkite_utils.so: kite_dynamics/CMakeFiles/kite_utils.dir/build.make
kite_dynamics/libkite_utils.so: kite_dynamics/CMakeFiles/kite_utils.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/haochengli/KiteEnergySystems/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libkite_utils.so"
	cd /home/haochengli/KiteEnergySystems/build/kite_dynamics && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kite_utils.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
kite_dynamics/CMakeFiles/kite_utils.dir/build: kite_dynamics/libkite_utils.so

.PHONY : kite_dynamics/CMakeFiles/kite_utils.dir/build

kite_dynamics/CMakeFiles/kite_utils.dir/clean:
	cd /home/haochengli/KiteEnergySystems/build/kite_dynamics && $(CMAKE_COMMAND) -P CMakeFiles/kite_utils.dir/cmake_clean.cmake
.PHONY : kite_dynamics/CMakeFiles/kite_utils.dir/clean

kite_dynamics/CMakeFiles/kite_utils.dir/depend:
	cd /home/haochengli/KiteEnergySystems/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/haochengli/KiteEnergySystems /home/haochengli/KiteEnergySystems/kite_dynamics /home/haochengli/KiteEnergySystems/build /home/haochengli/KiteEnergySystems/build/kite_dynamics /home/haochengli/KiteEnergySystems/build/kite_dynamics/CMakeFiles/kite_utils.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : kite_dynamics/CMakeFiles/kite_utils.dir/depend

