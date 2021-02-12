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
include CMakeFiles/Actuators.MCD4R.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Actuators.MCD4R.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Actuators.MCD4R.dir/flags.make

CMakeFiles/Actuators.MCD4R.dir/src/Actuators/MCD4R/Task.cpp.o: CMakeFiles/Actuators.MCD4R.dir/flags.make
CMakeFiles/Actuators.MCD4R.dir/src/Actuators/MCD4R/Task.cpp.o: ../src/Actuators/MCD4R/Task.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Actuators.MCD4R.dir/src/Actuators/MCD4R/Task.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Actuators::MCD4R::Task, ActuatorsMCD4RTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -o CMakeFiles/Actuators.MCD4R.dir/src/Actuators/MCD4R/Task.cpp.o -c /home/parallels/CLionProjects/dune/src/Actuators/MCD4R/Task.cpp

CMakeFiles/Actuators.MCD4R.dir/src/Actuators/MCD4R/Task.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Actuators.MCD4R.dir/src/Actuators/MCD4R/Task.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Actuators::MCD4R::Task, ActuatorsMCD4RTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -E /home/parallels/CLionProjects/dune/src/Actuators/MCD4R/Task.cpp > CMakeFiles/Actuators.MCD4R.dir/src/Actuators/MCD4R/Task.cpp.i

CMakeFiles/Actuators.MCD4R.dir/src/Actuators/MCD4R/Task.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Actuators.MCD4R.dir/src/Actuators/MCD4R/Task.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Actuators::MCD4R::Task, ActuatorsMCD4RTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -S /home/parallels/CLionProjects/dune/src/Actuators/MCD4R/Task.cpp -o CMakeFiles/Actuators.MCD4R.dir/src/Actuators/MCD4R/Task.cpp.s

# Object files for target Actuators.MCD4R
Actuators_MCD4R_OBJECTS = \
"CMakeFiles/Actuators.MCD4R.dir/src/Actuators/MCD4R/Task.cpp.o"

# External object files for target Actuators.MCD4R
Actuators_MCD4R_EXTERNAL_OBJECTS =

libActuators.MCD4R.a: CMakeFiles/Actuators.MCD4R.dir/src/Actuators/MCD4R/Task.cpp.o
libActuators.MCD4R.a: CMakeFiles/Actuators.MCD4R.dir/build.make
libActuators.MCD4R.a: CMakeFiles/Actuators.MCD4R.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libActuators.MCD4R.a"
	$(CMAKE_COMMAND) -P CMakeFiles/Actuators.MCD4R.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Actuators.MCD4R.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Actuators.MCD4R.dir/build: libActuators.MCD4R.a

.PHONY : CMakeFiles/Actuators.MCD4R.dir/build

CMakeFiles/Actuators.MCD4R.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Actuators.MCD4R.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Actuators.MCD4R.dir/clean

CMakeFiles/Actuators.MCD4R.dir/depend:
	cd /home/parallels/CLionProjects/dune/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/parallels/CLionProjects/dune /home/parallels/CLionProjects/dune /home/parallels/CLionProjects/dune/cmake-build-debug /home/parallels/CLionProjects/dune/cmake-build-debug /home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles/Actuators.MCD4R.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Actuators.MCD4R.dir/depend

