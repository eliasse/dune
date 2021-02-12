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
include CMakeFiles/Autonomy.TextActions.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Autonomy.TextActions.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Autonomy.TextActions.dir/flags.make

CMakeFiles/Autonomy.TextActions.dir/src/Autonomy/TextActions/Task.cpp.o: CMakeFiles/Autonomy.TextActions.dir/flags.make
CMakeFiles/Autonomy.TextActions.dir/src/Autonomy/TextActions/Task.cpp.o: ../src/Autonomy/TextActions/Task.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Autonomy.TextActions.dir/src/Autonomy/TextActions/Task.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Autonomy::TextActions::Task, AutonomyTextActionsTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -o CMakeFiles/Autonomy.TextActions.dir/src/Autonomy/TextActions/Task.cpp.o -c /home/parallels/CLionProjects/dune/src/Autonomy/TextActions/Task.cpp

CMakeFiles/Autonomy.TextActions.dir/src/Autonomy/TextActions/Task.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Autonomy.TextActions.dir/src/Autonomy/TextActions/Task.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Autonomy::TextActions::Task, AutonomyTextActionsTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -E /home/parallels/CLionProjects/dune/src/Autonomy/TextActions/Task.cpp > CMakeFiles/Autonomy.TextActions.dir/src/Autonomy/TextActions/Task.cpp.i

CMakeFiles/Autonomy.TextActions.dir/src/Autonomy/TextActions/Task.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Autonomy.TextActions.dir/src/Autonomy/TextActions/Task.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -DDUNE_TASK="DUNE_TASK_EXPORT(::Autonomy::TextActions::Task, AutonomyTextActionsTask)"  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -Wall -Wshadow -pedantic -S /home/parallels/CLionProjects/dune/src/Autonomy/TextActions/Task.cpp -o CMakeFiles/Autonomy.TextActions.dir/src/Autonomy/TextActions/Task.cpp.s

# Object files for target Autonomy.TextActions
Autonomy_TextActions_OBJECTS = \
"CMakeFiles/Autonomy.TextActions.dir/src/Autonomy/TextActions/Task.cpp.o"

# External object files for target Autonomy.TextActions
Autonomy_TextActions_EXTERNAL_OBJECTS =

libAutonomy.TextActions.a: CMakeFiles/Autonomy.TextActions.dir/src/Autonomy/TextActions/Task.cpp.o
libAutonomy.TextActions.a: CMakeFiles/Autonomy.TextActions.dir/build.make
libAutonomy.TextActions.a: CMakeFiles/Autonomy.TextActions.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libAutonomy.TextActions.a"
	$(CMAKE_COMMAND) -P CMakeFiles/Autonomy.TextActions.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Autonomy.TextActions.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Autonomy.TextActions.dir/build: libAutonomy.TextActions.a

.PHONY : CMakeFiles/Autonomy.TextActions.dir/build

CMakeFiles/Autonomy.TextActions.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Autonomy.TextActions.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Autonomy.TextActions.dir/clean

CMakeFiles/Autonomy.TextActions.dir/depend:
	cd /home/parallels/CLionProjects/dune/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/parallels/CLionProjects/dune /home/parallels/CLionProjects/dune /home/parallels/CLionProjects/dune/cmake-build-debug /home/parallels/CLionProjects/dune/cmake-build-debug /home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles/Autonomy.TextActions.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Autonomy.TextActions.dir/depend

