# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/ga63xaf/angelina

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ga63xaf/angelina/build

# Utility rule file for referee-doc.

# Include the progress variables for this target.
include hermes/CMakeFiles/referee-doc.dir/progress.make

hermes/CMakeFiles/referee-doc:
	cd /home/ga63xaf/angelina/build/hermes && /usr/bin/doxygen /home/ga63xaf/angelina/build/hermes/referee.doxy

referee-doc: hermes/CMakeFiles/referee-doc
referee-doc: hermes/CMakeFiles/referee-doc.dir/build.make
.PHONY : referee-doc

# Rule to build all files generated by this target.
hermes/CMakeFiles/referee-doc.dir/build: referee-doc
.PHONY : hermes/CMakeFiles/referee-doc.dir/build

hermes/CMakeFiles/referee-doc.dir/clean:
	cd /home/ga63xaf/angelina/build/hermes && $(CMAKE_COMMAND) -P CMakeFiles/referee-doc.dir/cmake_clean.cmake
.PHONY : hermes/CMakeFiles/referee-doc.dir/clean

hermes/CMakeFiles/referee-doc.dir/depend:
	cd /home/ga63xaf/angelina/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ga63xaf/angelina /home/ga63xaf/angelina/hermes /home/ga63xaf/angelina/build /home/ga63xaf/angelina/build/hermes /home/ga63xaf/angelina/build/hermes/CMakeFiles/referee-doc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : hermes/CMakeFiles/referee-doc.dir/depend
