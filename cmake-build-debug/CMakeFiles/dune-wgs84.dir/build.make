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
include CMakeFiles/dune-wgs84.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/dune-wgs84.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/dune-wgs84.dir/flags.make

CMakeFiles/dune-wgs84.dir/programs/utils/dune-wgs84.cpp.o: CMakeFiles/dune-wgs84.dir/flags.make
CMakeFiles/dune-wgs84.dir/programs/utils/dune-wgs84.cpp.o: ../programs/utils/dune-wgs84.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/dune-wgs84.dir/programs/utils/dune-wgs84.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dune-wgs84.dir/programs/utils/dune-wgs84.cpp.o -c /home/parallels/CLionProjects/dune/programs/utils/dune-wgs84.cpp

CMakeFiles/dune-wgs84.dir/programs/utils/dune-wgs84.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dune-wgs84.dir/programs/utils/dune-wgs84.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/parallels/CLionProjects/dune/programs/utils/dune-wgs84.cpp > CMakeFiles/dune-wgs84.dir/programs/utils/dune-wgs84.cpp.i

CMakeFiles/dune-wgs84.dir/programs/utils/dune-wgs84.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dune-wgs84.dir/programs/utils/dune-wgs84.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/parallels/CLionProjects/dune/programs/utils/dune-wgs84.cpp -o CMakeFiles/dune-wgs84.dir/programs/utils/dune-wgs84.cpp.s

# Object files for target dune-wgs84
dune__wgs84_OBJECTS = \
"CMakeFiles/dune-wgs84.dir/programs/utils/dune-wgs84.cpp.o"

# External object files for target dune-wgs84
dune__wgs84_EXTERNAL_OBJECTS =

dune-wgs84: CMakeFiles/dune-wgs84.dir/programs/utils/dune-wgs84.cpp.o
dune-wgs84: CMakeFiles/dune-wgs84.dir/build.make
dune-wgs84: libdune-core.a
dune-wgs84: CMakeFiles/dune-wgs84.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable dune-wgs84"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dune-wgs84.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/dune-wgs84.dir/build: dune-wgs84

.PHONY : CMakeFiles/dune-wgs84.dir/build

CMakeFiles/dune-wgs84.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dune-wgs84.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dune-wgs84.dir/clean

CMakeFiles/dune-wgs84.dir/depend:
	cd /home/parallels/CLionProjects/dune/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/parallels/CLionProjects/dune /home/parallels/CLionProjects/dune /home/parallels/CLionProjects/dune/cmake-build-debug /home/parallels/CLionProjects/dune/cmake-build-debug /home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles/dune-wgs84.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/dune-wgs84.dir/depend

