# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/cfe

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1

# Include any dependencies generated for this target.
include apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/depend.make

# Include the progress variables for this target.
include apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/progress.make

# Include the compile flags for this target's objects.
include apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/flags.make

apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/fsw/src/gpsnodemcu_lib.c.o: apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/flags.make
apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/fsw/src/gpsnodemcu_lib.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/gpsnodemcu_lib/fsw/src/gpsnodemcu_lib.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/fsw/src/gpsnodemcu_lib.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/gpsnodemcu_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/gpsnodemcu_lib.dir/fsw/src/gpsnodemcu_lib.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/gpsnodemcu_lib/fsw/src/gpsnodemcu_lib.c

apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/fsw/src/gpsnodemcu_lib.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/gpsnodemcu_lib.dir/fsw/src/gpsnodemcu_lib.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/gpsnodemcu_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/gpsnodemcu_lib/fsw/src/gpsnodemcu_lib.c > CMakeFiles/gpsnodemcu_lib.dir/fsw/src/gpsnodemcu_lib.c.i

apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/fsw/src/gpsnodemcu_lib.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/gpsnodemcu_lib.dir/fsw/src/gpsnodemcu_lib.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/gpsnodemcu_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/gpsnodemcu_lib/fsw/src/gpsnodemcu_lib.c -o CMakeFiles/gpsnodemcu_lib.dir/fsw/src/gpsnodemcu_lib.c.s

# Object files for target gpsnodemcu_lib
gpsnodemcu_lib_OBJECTS = \
"CMakeFiles/gpsnodemcu_lib.dir/fsw/src/gpsnodemcu_lib.c.o"

# External object files for target gpsnodemcu_lib
gpsnodemcu_lib_EXTERNAL_OBJECTS =

apps/gpsnodemcu_lib/gpsnodemcu_lib.so: apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/fsw/src/gpsnodemcu_lib.c.o
apps/gpsnodemcu_lib/gpsnodemcu_lib.so: apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/build.make
apps/gpsnodemcu_lib/gpsnodemcu_lib.so: apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C shared module gpsnodemcu_lib.so"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/gpsnodemcu_lib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gpsnodemcu_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/build: apps/gpsnodemcu_lib/gpsnodemcu_lib.so

.PHONY : apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/build

apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/clean:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/gpsnodemcu_lib && $(CMAKE_COMMAND) -P CMakeFiles/gpsnodemcu_lib.dir/cmake_clean.cmake
.PHONY : apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/clean

apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/depend:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/cfe /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/gpsnodemcu_lib /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/gpsnodemcu_lib /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/gpsnodemcu_lib/CMakeFiles/gpsnodemcu_lib.dir/depend

