# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

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
CMAKE_COMMAND = /home/parallels/clion-2020.1.1/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/parallels/clion-2020.1.1/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/parallels/CLionProjects/dune

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/parallels/CLionProjects/dune/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/compass_calibration.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/compass_calibration.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/compass_calibration.dir/flags.make

CMakeFiles/compass_calibration.dir/programs/compass_calibration.cpp.o: CMakeFiles/compass_calibration.dir/flags.make
CMakeFiles/compass_calibration.dir/programs/compass_calibration.cpp.o: ../programs/compass_calibration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/compass_calibration.dir/programs/compass_calibration.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/compass_calibration.dir/programs/compass_calibration.cpp.o -c /home/parallels/CLionProjects/dune/programs/compass_calibration.cpp

CMakeFiles/compass_calibration.dir/programs/compass_calibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/compass_calibration.dir/programs/compass_calibration.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/parallels/CLionProjects/dune/programs/compass_calibration.cpp > CMakeFiles/compass_calibration.dir/programs/compass_calibration.cpp.i

CMakeFiles/compass_calibration.dir/programs/compass_calibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/compass_calibration.dir/programs/compass_calibration.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/parallels/CLionProjects/dune/programs/compass_calibration.cpp -o CMakeFiles/compass_calibration.dir/programs/compass_calibration.cpp.s

# Object files for target compass_calibration
compass_calibration_OBJECTS = \
"CMakeFiles/compass_calibration.dir/programs/compass_calibration.cpp.o"

# External object files for target compass_calibration
compass_calibration_EXTERNAL_OBJECTS =

compass_calibration: CMakeFiles/compass_calibration.dir/programs/compass_calibration.cpp.o
compass_calibration: CMakeFiles/compass_calibration.dir/build.make
compass_calibration: libdune-core.a
compass_calibration: CMakeFiles/compass_calibration.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable compass_calibration"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/compass_calibration.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/compass_calibration.dir/build: compass_calibration

.PHONY : CMakeFiles/compass_calibration.dir/build

CMakeFiles/compass_calibration.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/compass_calibration.dir/cmake_clean.cmake
.PHONY : CMakeFiles/compass_calibration.dir/clean

CMakeFiles/compass_calibration.dir/depend:
	cd /home/parallels/CLionProjects/dune/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/parallels/CLionProjects/dune /home/parallels/CLionProjects/dune /home/parallels/CLionProjects/dune/cmake-build-debug /home/parallels/CLionProjects/dune/cmake-build-debug /home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles/compass_calibration.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/compass_calibration.dir/depend

