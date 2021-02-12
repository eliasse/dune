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
include CMakeFiles/Simulators.GPS.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Simulators.GPS.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Simulators.GPS.dir/flags.make

CMakeFiles/Simulators.GPS.dir/src/Simulators/GPS/Task.cpp.o: CMakeFiles/Simulators.GPS.dir/flags.make
CMakeFiles/Simulators.GPS.dir/src/Simulators/GPS/Task.cpp.o: ../src/Simulators/GPS/Task.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Simulators.GPS.dir/src/Simulators/GPS/Task.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Simulators::GPS::Task, SimulatorsGPSTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -o CMakeFiles/Simulators.GPS.dir/src/Simulators/GPS/Task.cpp.o -c /home/parallels/CLionProjects/dune/src/Simulators/GPS/Task.cpp

CMakeFiles/Simulators.GPS.dir/src/Simulators/GPS/Task.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Simulators.GPS.dir/src/Simulators/GPS/Task.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Simulators::GPS::Task, SimulatorsGPSTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -E /home/parallels/CLionProjects/dune/src/Simulators/GPS/Task.cpp > CMakeFiles/Simulators.GPS.dir/src/Simulators/GPS/Task.cpp.i

CMakeFiles/Simulators.GPS.dir/src/Simulators/GPS/Task.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Simulators.GPS.dir/src/Simulators/GPS/Task.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Simulators::GPS::Task, SimulatorsGPSTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -S /home/parallels/CLionProjects/dune/src/Simulators/GPS/Task.cpp -o CMakeFiles/Simulators.GPS.dir/src/Simulators/GPS/Task.cpp.s

# Object files for target Simulators.GPS
Simulators_GPS_OBJECTS = \
"CMakeFiles/Simulators.GPS.dir/src/Simulators/GPS/Task.cpp.o"

# External object files for target Simulators.GPS
Simulators_GPS_EXTERNAL_OBJECTS =

libSimulators.GPS.a: CMakeFiles/Simulators.GPS.dir/src/Simulators/GPS/Task.cpp.o
libSimulators.GPS.a: CMakeFiles/Simulators.GPS.dir/build.make
libSimulators.GPS.a: CMakeFiles/Simulators.GPS.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libSimulators.GPS.a"
	$(CMAKE_COMMAND) -P CMakeFiles/Simulators.GPS.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Simulators.GPS.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Simulators.GPS.dir/build: libSimulators.GPS.a

.PHONY : CMakeFiles/Simulators.GPS.dir/build

CMakeFiles/Simulators.GPS.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Simulators.GPS.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Simulators.GPS.dir/clean

CMakeFiles/Simulators.GPS.dir/depend:
	cd /home/parallels/CLionProjects/dune/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/parallels/CLionProjects/dune /home/parallels/CLionProjects/dune /home/parallels/CLionProjects/dune/cmake-build-debug /home/parallels/CLionProjects/dune/cmake-build-debug /home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles/Simulators.GPS.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Simulators.GPS.dir/depend

