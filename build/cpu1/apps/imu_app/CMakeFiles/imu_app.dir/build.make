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
include apps/imu_app/CMakeFiles/imu_app.dir/depend.make

# Include the progress variables for this target.
include apps/imu_app/CMakeFiles/imu_app.dir/progress.make

# Include the compile flags for this target's objects.
include apps/imu_app/CMakeFiles/imu_app.dir/flags.make

apps/imu_app/CMakeFiles/imu_app.dir/fsw/src/imu_app.c.o: apps/imu_app/CMakeFiles/imu_app.dir/flags.make
apps/imu_app/CMakeFiles/imu_app.dir/fsw/src/imu_app.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/imu_app/fsw/src/imu_app.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object apps/imu_app/CMakeFiles/imu_app.dir/fsw/src/imu_app.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/imu_app && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/imu_app.dir/fsw/src/imu_app.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/imu_app/fsw/src/imu_app.c

apps/imu_app/CMakeFiles/imu_app.dir/fsw/src/imu_app.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/imu_app.dir/fsw/src/imu_app.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/imu_app && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/imu_app/fsw/src/imu_app.c > CMakeFiles/imu_app.dir/fsw/src/imu_app.c.i

apps/imu_app/CMakeFiles/imu_app.dir/fsw/src/imu_app.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/imu_app.dir/fsw/src/imu_app.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/imu_app && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/imu_app/fsw/src/imu_app.c -o CMakeFiles/imu_app.dir/fsw/src/imu_app.c.s

# Object files for target imu_app
imu_app_OBJECTS = \
"CMakeFiles/imu_app.dir/fsw/src/imu_app.c.o"

# External object files for target imu_app
imu_app_EXTERNAL_OBJECTS =

apps/imu_app/imu_app.so: apps/imu_app/CMakeFiles/imu_app.dir/fsw/src/imu_app.c.o
apps/imu_app/imu_app.so: apps/imu_app/CMakeFiles/imu_app.dir/build.make
apps/imu_app/imu_app.so: apps/imu_app/CMakeFiles/imu_app.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C shared module imu_app.so"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/imu_app && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/imu_app.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/imu_app/CMakeFiles/imu_app.dir/build: apps/imu_app/imu_app.so

.PHONY : apps/imu_app/CMakeFiles/imu_app.dir/build

apps/imu_app/CMakeFiles/imu_app.dir/clean:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/imu_app && $(CMAKE_COMMAND) -P CMakeFiles/imu_app.dir/cmake_clean.cmake
.PHONY : apps/imu_app/CMakeFiles/imu_app.dir/clean

apps/imu_app/CMakeFiles/imu_app.dir/depend:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/cfe /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/imu_app /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/imu_app /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/imu_app/CMakeFiles/imu_app.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/imu_app/CMakeFiles/imu_app.dir/depend

