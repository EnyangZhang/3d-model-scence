# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/cosc/student/ezh15/Desktop/Assignment1/code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cosc/student/ezh15/Desktop/Assignment1/build-code-Desktop-Default

# Include any dependencies generated for this target.
include CMakeFiles/Assignment1.out.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Assignment1.out.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Assignment1.out.dir/flags.make

CMakeFiles/Assignment1.out.dir/Assignment1.cpp.o: CMakeFiles/Assignment1.out.dir/flags.make
CMakeFiles/Assignment1.out.dir/Assignment1.cpp.o: /home/cosc/student/ezh15/Desktop/Assignment1/code/Assignment1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cosc/student/ezh15/Desktop/Assignment1/build-code-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Assignment1.out.dir/Assignment1.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Assignment1.out.dir/Assignment1.cpp.o -c /home/cosc/student/ezh15/Desktop/Assignment1/code/Assignment1.cpp

CMakeFiles/Assignment1.out.dir/Assignment1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Assignment1.out.dir/Assignment1.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cosc/student/ezh15/Desktop/Assignment1/code/Assignment1.cpp > CMakeFiles/Assignment1.out.dir/Assignment1.cpp.i

CMakeFiles/Assignment1.out.dir/Assignment1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Assignment1.out.dir/Assignment1.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cosc/student/ezh15/Desktop/Assignment1/code/Assignment1.cpp -o CMakeFiles/Assignment1.out.dir/Assignment1.cpp.s

CMakeFiles/Assignment1.out.dir/Assignment1.cpp.o.requires:

.PHONY : CMakeFiles/Assignment1.out.dir/Assignment1.cpp.o.requires

CMakeFiles/Assignment1.out.dir/Assignment1.cpp.o.provides: CMakeFiles/Assignment1.out.dir/Assignment1.cpp.o.requires
	$(MAKE) -f CMakeFiles/Assignment1.out.dir/build.make CMakeFiles/Assignment1.out.dir/Assignment1.cpp.o.provides.build
.PHONY : CMakeFiles/Assignment1.out.dir/Assignment1.cpp.o.provides

CMakeFiles/Assignment1.out.dir/Assignment1.cpp.o.provides.build: CMakeFiles/Assignment1.out.dir/Assignment1.cpp.o


# Object files for target Assignment1.out
Assignment1_out_OBJECTS = \
"CMakeFiles/Assignment1.out.dir/Assignment1.cpp.o"

# External object files for target Assignment1.out
Assignment1_out_EXTERNAL_OBJECTS =

Assignment1.out: CMakeFiles/Assignment1.out.dir/Assignment1.cpp.o
Assignment1.out: CMakeFiles/Assignment1.out.dir/build.make
Assignment1.out: /usr/lib/x86_64-linux-gnu/libGL.so
Assignment1.out: /usr/lib/x86_64-linux-gnu/libGLU.so
Assignment1.out: /usr/lib/x86_64-linux-gnu/libglut.so
Assignment1.out: /usr/lib/x86_64-linux-gnu/libXmu.so
Assignment1.out: /usr/lib/x86_64-linux-gnu/libXi.so
Assignment1.out: CMakeFiles/Assignment1.out.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cosc/student/ezh15/Desktop/Assignment1/build-code-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Assignment1.out"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Assignment1.out.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Assignment1.out.dir/build: Assignment1.out

.PHONY : CMakeFiles/Assignment1.out.dir/build

CMakeFiles/Assignment1.out.dir/requires: CMakeFiles/Assignment1.out.dir/Assignment1.cpp.o.requires

.PHONY : CMakeFiles/Assignment1.out.dir/requires

CMakeFiles/Assignment1.out.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Assignment1.out.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Assignment1.out.dir/clean

CMakeFiles/Assignment1.out.dir/depend:
	cd /home/cosc/student/ezh15/Desktop/Assignment1/build-code-Desktop-Default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cosc/student/ezh15/Desktop/Assignment1/code /home/cosc/student/ezh15/Desktop/Assignment1/code /home/cosc/student/ezh15/Desktop/Assignment1/build-code-Desktop-Default /home/cosc/student/ezh15/Desktop/Assignment1/build-code-Desktop-Default /home/cosc/student/ezh15/Desktop/Assignment1/build-code-Desktop-Default/CMakeFiles/Assignment1.out.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Assignment1.out.dir/depend

