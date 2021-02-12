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
include CMakeFiles/Sensors.SADC.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Sensors.SADC.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Sensors.SADC.dir/flags.make

CMakeFiles/Sensors.SADC.dir/src/Sensors/SADC/Task.cpp.o: CMakeFiles/Sensors.SADC.dir/flags.make
CMakeFiles/Sensors.SADC.dir/src/Sensors/SADC/Task.cpp.o: ../src/Sensors/SADC/Task.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Sensors.SADC.dir/src/Sensors/SADC/Task.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Sensors::SADC::Task, SensorsSADCTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -o CMakeFiles/Sensors.SADC.dir/src/Sensors/SADC/Task.cpp.o -c /home/parallels/CLionProjects/dune/src/Sensors/SADC/Task.cpp

CMakeFiles/Sensors.SADC.dir/src/Sensors/SADC/Task.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Sensors.SADC.dir/src/Sensors/SADC/Task.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Sensors::SADC::Task, SensorsSADCTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -E /home/parallels/CLionProjects/dune/src/Sensors/SADC/Task.cpp > CMakeFiles/Sensors.SADC.dir/src/Sensors/SADC/Task.cpp.i

CMakeFiles/Sensors.SADC.dir/src/Sensors/SADC/Task.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Sensors.SADC.dir/src/Sensors/SADC/Task.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Sensors::SADC::Task, SensorsSADCTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -S /home/parallels/CLionProjects/dune/src/Sensors/SADC/Task.cpp -o CMakeFiles/Sensors.SADC.dir/src/Sensors/SADC/Task.cpp.s

# Object files for target Sensors.SADC
Sensors_SADC_OBJECTS = \
"CMakeFiles/Sensors.SADC.dir/src/Sensors/SADC/Task.cpp.o"

# External object files for target Sensors.SADC
Sensors_SADC_EXTERNAL_OBJECTS =

libSensors.SADC.a: CMakeFiles/Sensors.SADC.dir/src/Sensors/SADC/Task.cpp.o
libSensors.SADC.a: CMakeFiles/Sensors.SADC.dir/build.make
libSensors.SADC.a: CMakeFiles/Sensors.SADC.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libSensors.SADC.a"
	$(CMAKE_COMMAND) -P CMakeFiles/Sensors.SADC.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Sensors.SADC.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Sensors.SADC.dir/build: libSensors.SADC.a

.PHONY : CMakeFiles/Sensors.SADC.dir/build

CMakeFiles/Sensors.SADC.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Sensors.SADC.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Sensors.SADC.dir/clean

CMakeFiles/Sensors.SADC.dir/depend:
	cd /home/parallels/CLionProjects/dune/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/parallels/CLionProjects/dune /home/parallels/CLionProjects/dune /home/parallels/CLionProjects/dune/cmake-build-debug /home/parallels/CLionProjects/dune/cmake-build-debug /home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles/Sensors.SADC.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Sensors.SADC.dir/depend

