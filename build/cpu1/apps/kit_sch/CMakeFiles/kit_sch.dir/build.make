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
include apps/kit_sch/CMakeFiles/kit_sch.dir/depend.make

# Include the progress variables for this target.
include apps/kit_sch/CMakeFiles/kit_sch.dir/progress.make

# Include the compile flags for this target's objects.
include apps/kit_sch/CMakeFiles/kit_sch.dir/flags.make

apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/kit_sch_app.c.o: apps/kit_sch/CMakeFiles/kit_sch.dir/flags.make
apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/kit_sch_app.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/kit_sch/fsw/src/kit_sch_app.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/kit_sch_app.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_sch && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/kit_sch.dir/fsw/src/kit_sch_app.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/kit_sch/fsw/src/kit_sch_app.c

apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/kit_sch_app.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/kit_sch.dir/fsw/src/kit_sch_app.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_sch && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/kit_sch/fsw/src/kit_sch_app.c > CMakeFiles/kit_sch.dir/fsw/src/kit_sch_app.c.i

apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/kit_sch_app.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/kit_sch.dir/fsw/src/kit_sch_app.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_sch && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/kit_sch/fsw/src/kit_sch_app.c -o CMakeFiles/kit_sch.dir/fsw/src/kit_sch_app.c.s

apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/msgtbl.c.o: apps/kit_sch/CMakeFiles/kit_sch.dir/flags.make
apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/msgtbl.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/kit_sch/fsw/src/msgtbl.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/msgtbl.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_sch && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/kit_sch.dir/fsw/src/msgtbl.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/kit_sch/fsw/src/msgtbl.c

apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/msgtbl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/kit_sch.dir/fsw/src/msgtbl.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_sch && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/kit_sch/fsw/src/msgtbl.c > CMakeFiles/kit_sch.dir/fsw/src/msgtbl.c.i

apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/msgtbl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/kit_sch.dir/fsw/src/msgtbl.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_sch && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/kit_sch/fsw/src/msgtbl.c -o CMakeFiles/kit_sch.dir/fsw/src/msgtbl.c.s

apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/scheduler.c.o: apps/kit_sch/CMakeFiles/kit_sch.dir/flags.make
apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/scheduler.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/kit_sch/fsw/src/scheduler.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/scheduler.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_sch && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/kit_sch.dir/fsw/src/scheduler.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/kit_sch/fsw/src/scheduler.c

apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/scheduler.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/kit_sch.dir/fsw/src/scheduler.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_sch && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/kit_sch/fsw/src/scheduler.c > CMakeFiles/kit_sch.dir/fsw/src/scheduler.c.i

apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/scheduler.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/kit_sch.dir/fsw/src/scheduler.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_sch && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/kit_sch/fsw/src/scheduler.c -o CMakeFiles/kit_sch.dir/fsw/src/scheduler.c.s

apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/schtbl.c.o: apps/kit_sch/CMakeFiles/kit_sch.dir/flags.make
apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/schtbl.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/kit_sch/fsw/src/schtbl.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/schtbl.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_sch && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/kit_sch.dir/fsw/src/schtbl.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/kit_sch/fsw/src/schtbl.c

apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/schtbl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/kit_sch.dir/fsw/src/schtbl.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_sch && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/kit_sch/fsw/src/schtbl.c > CMakeFiles/kit_sch.dir/fsw/src/schtbl.c.i

apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/schtbl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/kit_sch.dir/fsw/src/schtbl.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_sch && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/kit_sch/fsw/src/schtbl.c -o CMakeFiles/kit_sch.dir/fsw/src/schtbl.c.s

# Object files for target kit_sch
kit_sch_OBJECTS = \
"CMakeFiles/kit_sch.dir/fsw/src/kit_sch_app.c.o" \
"CMakeFiles/kit_sch.dir/fsw/src/msgtbl.c.o" \
"CMakeFiles/kit_sch.dir/fsw/src/scheduler.c.o" \
"CMakeFiles/kit_sch.dir/fsw/src/schtbl.c.o"

# External object files for target kit_sch
kit_sch_EXTERNAL_OBJECTS =

apps/kit_sch/kit_sch.so: apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/kit_sch_app.c.o
apps/kit_sch/kit_sch.so: apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/msgtbl.c.o
apps/kit_sch/kit_sch.so: apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/scheduler.c.o
apps/kit_sch/kit_sch.so: apps/kit_sch/CMakeFiles/kit_sch.dir/fsw/src/schtbl.c.o
apps/kit_sch/kit_sch.so: apps/kit_sch/CMakeFiles/kit_sch.dir/build.make
apps/kit_sch/kit_sch.so: apps/kit_sch/CMakeFiles/kit_sch.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking C shared module kit_sch.so"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_sch && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kit_sch.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/kit_sch/CMakeFiles/kit_sch.dir/build: apps/kit_sch/kit_sch.so

.PHONY : apps/kit_sch/CMakeFiles/kit_sch.dir/build

apps/kit_sch/CMakeFiles/kit_sch.dir/clean:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_sch && $(CMAKE_COMMAND) -P CMakeFiles/kit_sch.dir/cmake_clean.cmake
.PHONY : apps/kit_sch/CMakeFiles/kit_sch.dir/clean

apps/kit_sch/CMakeFiles/kit_sch.dir/depend:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/cfe /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/kit_sch /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_sch /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/kit_sch/CMakeFiles/kit_sch.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/kit_sch/CMakeFiles/kit_sch.dir/depend

