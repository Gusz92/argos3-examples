# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_COMMAND = /opt/clion-2016.3.2/bin/cmake/bin/cmake

# The command to remove a file.
RM = /opt/clion-2016.3.2/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/gx535/CLionProjects/argos3-examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gx535/CLionProjects/argos3-examples/build

# Include any dependencies generated for this target.
include controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/depend.make

# Include the progress variables for this target.
include controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/progress.make

# Include the compile flags for this target's objects.
include controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/flags.make

controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.o: controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/flags.make
controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.o: ../controllers/footbot_foraging/footbot_foraging.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gx535/CLionProjects/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.o"
	cd /home/gx535/CLionProjects/argos3-examples/build/controllers/footbot_foraging && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.o -c /home/gx535/CLionProjects/argos3-examples/controllers/footbot_foraging/footbot_foraging.cpp

controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.i"
	cd /home/gx535/CLionProjects/argos3-examples/build/controllers/footbot_foraging && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gx535/CLionProjects/argos3-examples/controllers/footbot_foraging/footbot_foraging.cpp > CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.i

controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.s"
	cd /home/gx535/CLionProjects/argos3-examples/build/controllers/footbot_foraging && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gx535/CLionProjects/argos3-examples/controllers/footbot_foraging/footbot_foraging.cpp -o CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.s

controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.o.requires:

.PHONY : controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.o.requires

controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.o.provides: controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.o.requires
	$(MAKE) -f controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/build.make controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.o.provides.build
.PHONY : controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.o.provides

controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.o.provides.build: controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.o


# Object files for target footbot_foraging
footbot_foraging_OBJECTS = \
"CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.o"

# External object files for target footbot_foraging
footbot_foraging_EXTERNAL_OBJECTS =

controllers/footbot_foraging/libfootbot_foraging.so: controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.o
controllers/footbot_foraging/libfootbot_foraging.so: controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/build.make
controllers/footbot_foraging/libfootbot_foraging.so: controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gx535/CLionProjects/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libfootbot_foraging.so"
	cd /home/gx535/CLionProjects/argos3-examples/build/controllers/footbot_foraging && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/footbot_foraging.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/build: controllers/footbot_foraging/libfootbot_foraging.so

.PHONY : controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/build

controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/requires: controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/footbot_foraging.cpp.o.requires

.PHONY : controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/requires

controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/clean:
	cd /home/gx535/CLionProjects/argos3-examples/build/controllers/footbot_foraging && $(CMAKE_COMMAND) -P CMakeFiles/footbot_foraging.dir/cmake_clean.cmake
.PHONY : controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/clean

controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/depend:
	cd /home/gx535/CLionProjects/argos3-examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gx535/CLionProjects/argos3-examples /home/gx535/CLionProjects/argos3-examples/controllers/footbot_foraging /home/gx535/CLionProjects/argos3-examples/build /home/gx535/CLionProjects/argos3-examples/build/controllers/footbot_foraging /home/gx535/CLionProjects/argos3-examples/build/controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : controllers/footbot_foraging/CMakeFiles/footbot_foraging.dir/depend

