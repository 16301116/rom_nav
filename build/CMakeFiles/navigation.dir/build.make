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
CMAKE_SOURCE_DIR = /home/waihekor/45kg/ROM-NAV-Cmake-Nvidia

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/waihekor/45kg/ROM-NAV-Cmake-Nvidia/build

# Include any dependencies generated for this target.
include CMakeFiles/navigation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/navigation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/navigation.dir/flags.make

CMakeFiles/navigation.dir/apps/navigation.cpp.o: CMakeFiles/navigation.dir/flags.make
CMakeFiles/navigation.dir/apps/navigation.cpp.o: ../apps/navigation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/waihekor/45kg/ROM-NAV-Cmake-Nvidia/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/navigation.dir/apps/navigation.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/navigation.dir/apps/navigation.cpp.o -c /home/waihekor/45kg/ROM-NAV-Cmake-Nvidia/apps/navigation.cpp

CMakeFiles/navigation.dir/apps/navigation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/navigation.dir/apps/navigation.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/waihekor/45kg/ROM-NAV-Cmake-Nvidia/apps/navigation.cpp > CMakeFiles/navigation.dir/apps/navigation.cpp.i

CMakeFiles/navigation.dir/apps/navigation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/navigation.dir/apps/navigation.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/waihekor/45kg/ROM-NAV-Cmake-Nvidia/apps/navigation.cpp -o CMakeFiles/navigation.dir/apps/navigation.cpp.s

CMakeFiles/navigation.dir/apps/navigation.cpp.o.requires:

.PHONY : CMakeFiles/navigation.dir/apps/navigation.cpp.o.requires

CMakeFiles/navigation.dir/apps/navigation.cpp.o.provides: CMakeFiles/navigation.dir/apps/navigation.cpp.o.requires
	$(MAKE) -f CMakeFiles/navigation.dir/build.make CMakeFiles/navigation.dir/apps/navigation.cpp.o.provides.build
.PHONY : CMakeFiles/navigation.dir/apps/navigation.cpp.o.provides

CMakeFiles/navigation.dir/apps/navigation.cpp.o.provides.build: CMakeFiles/navigation.dir/apps/navigation.cpp.o


# Object files for target navigation
navigation_OBJECTS = \
"CMakeFiles/navigation.dir/apps/navigation.cpp.o"

# External object files for target navigation
navigation_EXTERNAL_OBJECTS =

navigation: CMakeFiles/navigation.dir/apps/navigation.cpp.o
navigation: CMakeFiles/navigation.dir/build.make
navigation: librom_nav.a
navigation: CMakeFiles/navigation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/waihekor/45kg/ROM-NAV-Cmake-Nvidia/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable navigation"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/navigation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/navigation.dir/build: navigation

.PHONY : CMakeFiles/navigation.dir/build

CMakeFiles/navigation.dir/requires: CMakeFiles/navigation.dir/apps/navigation.cpp.o.requires

.PHONY : CMakeFiles/navigation.dir/requires

CMakeFiles/navigation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/navigation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/navigation.dir/clean

CMakeFiles/navigation.dir/depend:
	cd /home/waihekor/45kg/ROM-NAV-Cmake-Nvidia/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/waihekor/45kg/ROM-NAV-Cmake-Nvidia /home/waihekor/45kg/ROM-NAV-Cmake-Nvidia /home/waihekor/45kg/ROM-NAV-Cmake-Nvidia/build /home/waihekor/45kg/ROM-NAV-Cmake-Nvidia/build /home/waihekor/45kg/ROM-NAV-Cmake-Nvidia/build/CMakeFiles/navigation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/navigation.dir/depend

