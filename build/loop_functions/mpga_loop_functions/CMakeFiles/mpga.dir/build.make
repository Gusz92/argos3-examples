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
include loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/depend.make

# Include the progress variables for this target.
include loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/progress.make

# Include the compile flags for this target's objects.
include loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/flags.make

loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga.cpp.o: loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/flags.make
loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga.cpp.o: ../loop_functions/mpga_loop_functions/mpga.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gx535/CLionProjects/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga.cpp.o"
	cd /home/gx535/CLionProjects/argos3-examples/build/loop_functions/mpga_loop_functions && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mpga.dir/mpga.cpp.o -c /home/gx535/CLionProjects/argos3-examples/loop_functions/mpga_loop_functions/mpga.cpp

loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpga.dir/mpga.cpp.i"
	cd /home/gx535/CLionProjects/argos3-examples/build/loop_functions/mpga_loop_functions && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gx535/CLionProjects/argos3-examples/loop_functions/mpga_loop_functions/mpga.cpp > CMakeFiles/mpga.dir/mpga.cpp.i

loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpga.dir/mpga.cpp.s"
	cd /home/gx535/CLionProjects/argos3-examples/build/loop_functions/mpga_loop_functions && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gx535/CLionProjects/argos3-examples/loop_functions/mpga_loop_functions/mpga.cpp -o CMakeFiles/mpga.dir/mpga.cpp.s

loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga.cpp.o.requires:

.PHONY : loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga.cpp.o.requires

loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga.cpp.o.provides: loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga.cpp.o.requires
	$(MAKE) -f loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/build.make loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga.cpp.o.provides.build
.PHONY : loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga.cpp.o.provides

loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga.cpp.o.provides.build: loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga.cpp.o


loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga_loop_functions.cpp.o: loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/flags.make
loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga_loop_functions.cpp.o: ../loop_functions/mpga_loop_functions/mpga_loop_functions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gx535/CLionProjects/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga_loop_functions.cpp.o"
	cd /home/gx535/CLionProjects/argos3-examples/build/loop_functions/mpga_loop_functions && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mpga.dir/mpga_loop_functions.cpp.o -c /home/gx535/CLionProjects/argos3-examples/loop_functions/mpga_loop_functions/mpga_loop_functions.cpp

loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga_loop_functions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpga.dir/mpga_loop_functions.cpp.i"
	cd /home/gx535/CLionProjects/argos3-examples/build/loop_functions/mpga_loop_functions && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gx535/CLionProjects/argos3-examples/loop_functions/mpga_loop_functions/mpga_loop_functions.cpp > CMakeFiles/mpga.dir/mpga_loop_functions.cpp.i

loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga_loop_functions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpga.dir/mpga_loop_functions.cpp.s"
	cd /home/gx535/CLionProjects/argos3-examples/build/loop_functions/mpga_loop_functions && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gx535/CLionProjects/argos3-examples/loop_functions/mpga_loop_functions/mpga_loop_functions.cpp -o CMakeFiles/mpga.dir/mpga_loop_functions.cpp.s

loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga_loop_functions.cpp.o.requires:

.PHONY : loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga_loop_functions.cpp.o.requires

loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga_loop_functions.cpp.o.provides: loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga_loop_functions.cpp.o.requires
	$(MAKE) -f loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/build.make loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga_loop_functions.cpp.o.provides.build
.PHONY : loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga_loop_functions.cpp.o.provides

loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga_loop_functions.cpp.o.provides.build: loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga_loop_functions.cpp.o


# Object files for target mpga
mpga_OBJECTS = \
"CMakeFiles/mpga.dir/mpga.cpp.o" \
"CMakeFiles/mpga.dir/mpga_loop_functions.cpp.o"

# External object files for target mpga
mpga_EXTERNAL_OBJECTS =

loop_functions/mpga_loop_functions/libmpga.so: loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga.cpp.o
loop_functions/mpga_loop_functions/libmpga.so: loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga_loop_functions.cpp.o
loop_functions/mpga_loop_functions/libmpga.so: loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/build.make
loop_functions/mpga_loop_functions/libmpga.so: loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gx535/CLionProjects/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library libmpga.so"
	cd /home/gx535/CLionProjects/argos3-examples/build/loop_functions/mpga_loop_functions && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mpga.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/build: loop_functions/mpga_loop_functions/libmpga.so

.PHONY : loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/build

loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/requires: loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga.cpp.o.requires
loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/requires: loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/mpga_loop_functions.cpp.o.requires

.PHONY : loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/requires

loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/clean:
	cd /home/gx535/CLionProjects/argos3-examples/build/loop_functions/mpga_loop_functions && $(CMAKE_COMMAND) -P CMakeFiles/mpga.dir/cmake_clean.cmake
.PHONY : loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/clean

loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/depend:
	cd /home/gx535/CLionProjects/argos3-examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gx535/CLionProjects/argos3-examples /home/gx535/CLionProjects/argos3-examples/loop_functions/mpga_loop_functions /home/gx535/CLionProjects/argos3-examples/build /home/gx535/CLionProjects/argos3-examples/build/loop_functions/mpga_loop_functions /home/gx535/CLionProjects/argos3-examples/build/loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : loop_functions/mpga_loop_functions/CMakeFiles/mpga.dir/depend

