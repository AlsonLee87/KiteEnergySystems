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
include kite_dynamics/CMakeFiles/EigenTest.dir/depend.make

# Include the progress variables for this target.
include kite_dynamics/CMakeFiles/EigenTest.dir/progress.make

# Include the compile flags for this target's objects.
include kite_dynamics/CMakeFiles/EigenTest.dir/flags.make

kite_dynamics/CMakeFiles/EigenTest.dir/test/EigenTest.cc.o: kite_dynamics/CMakeFiles/EigenTest.dir/flags.make
kite_dynamics/CMakeFiles/EigenTest.dir/test/EigenTest.cc.o: ../kite_dynamics/test/EigenTest.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/haochengli/KiteEnergySystems/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object kite_dynamics/CMakeFiles/EigenTest.dir/test/EigenTest.cc.o"
	cd /home/haochengli/KiteEnergySystems/build/kite_dynamics && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/EigenTest.dir/test/EigenTest.cc.o -c /home/haochengli/KiteEnergySystems/kite_dynamics/test/EigenTest.cc

kite_dynamics/CMakeFiles/EigenTest.dir/test/EigenTest.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/EigenTest.dir/test/EigenTest.cc.i"
	cd /home/haochengli/KiteEnergySystems/build/kite_dynamics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/haochengli/KiteEnergySystems/kite_dynamics/test/EigenTest.cc > CMakeFiles/EigenTest.dir/test/EigenTest.cc.i

kite_dynamics/CMakeFiles/EigenTest.dir/test/EigenTest.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/EigenTest.dir/test/EigenTest.cc.s"
	cd /home/haochengli/KiteEnergySystems/build/kite_dynamics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/haochengli/KiteEnergySystems/kite_dynamics/test/EigenTest.cc -o CMakeFiles/EigenTest.dir/test/EigenTest.cc.s

# Object files for target EigenTest
EigenTest_OBJECTS = \
"CMakeFiles/EigenTest.dir/test/EigenTest.cc.o"

# External object files for target EigenTest
EigenTest_EXTERNAL_OBJECTS =

kite_dynamics/EigenTest: kite_dynamics/CMakeFiles/EigenTest.dir/test/EigenTest.cc.o
kite_dynamics/EigenTest: kite_dynamics/CMakeFiles/EigenTest.dir/build.make
kite_dynamics/EigenTest: kite_dynamics/CMakeFiles/EigenTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/haochengli/KiteEnergySystems/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable EigenTest"
	cd /home/haochengli/KiteEnergySystems/build/kite_dynamics && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/EigenTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
kite_dynamics/CMakeFiles/EigenTest.dir/build: kite_dynamics/EigenTest

.PHONY : kite_dynamics/CMakeFiles/EigenTest.dir/build

kite_dynamics/CMakeFiles/EigenTest.dir/clean:
	cd /home/haochengli/KiteEnergySystems/build/kite_dynamics && $(CMAKE_COMMAND) -P CMakeFiles/EigenTest.dir/cmake_clean.cmake
.PHONY : kite_dynamics/CMakeFiles/EigenTest.dir/clean

kite_dynamics/CMakeFiles/EigenTest.dir/depend:
	cd /home/haochengli/KiteEnergySystems/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/haochengli/KiteEnergySystems /home/haochengli/KiteEnergySystems/kite_dynamics /home/haochengli/KiteEnergySystems/build /home/haochengli/KiteEnergySystems/build/kite_dynamics /home/haochengli/KiteEnergySystems/build/kite_dynamics/CMakeFiles/EigenTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : kite_dynamics/CMakeFiles/EigenTest.dir/depend

