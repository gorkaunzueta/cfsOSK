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
include apps/lc/CMakeFiles/lc.dir/depend.make

# Include the progress variables for this target.
include apps/lc/CMakeFiles/lc.dir/progress.make

# Include the compile flags for this target's objects.
include apps/lc/CMakeFiles/lc.dir/flags.make

apps/lc/CMakeFiles/lc.dir/fsw/src/lc_action.c.o: apps/lc/CMakeFiles/lc.dir/flags.make
apps/lc/CMakeFiles/lc.dir/fsw/src/lc_action.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_action.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object apps/lc/CMakeFiles/lc.dir/fsw/src/lc_action.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/lc.dir/fsw/src/lc_action.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_action.c

apps/lc/CMakeFiles/lc.dir/fsw/src/lc_action.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/lc.dir/fsw/src/lc_action.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_action.c > CMakeFiles/lc.dir/fsw/src/lc_action.c.i

apps/lc/CMakeFiles/lc.dir/fsw/src/lc_action.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/lc.dir/fsw/src/lc_action.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_action.c -o CMakeFiles/lc.dir/fsw/src/lc_action.c.s

apps/lc/CMakeFiles/lc.dir/fsw/src/lc_app.c.o: apps/lc/CMakeFiles/lc.dir/flags.make
apps/lc/CMakeFiles/lc.dir/fsw/src/lc_app.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_app.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object apps/lc/CMakeFiles/lc.dir/fsw/src/lc_app.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/lc.dir/fsw/src/lc_app.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_app.c

apps/lc/CMakeFiles/lc.dir/fsw/src/lc_app.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/lc.dir/fsw/src/lc_app.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_app.c > CMakeFiles/lc.dir/fsw/src/lc_app.c.i

apps/lc/CMakeFiles/lc.dir/fsw/src/lc_app.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/lc.dir/fsw/src/lc_app.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_app.c -o CMakeFiles/lc.dir/fsw/src/lc_app.c.s

apps/lc/CMakeFiles/lc.dir/fsw/src/lc_cmds.c.o: apps/lc/CMakeFiles/lc.dir/flags.make
apps/lc/CMakeFiles/lc.dir/fsw/src/lc_cmds.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_cmds.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object apps/lc/CMakeFiles/lc.dir/fsw/src/lc_cmds.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/lc.dir/fsw/src/lc_cmds.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_cmds.c

apps/lc/CMakeFiles/lc.dir/fsw/src/lc_cmds.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/lc.dir/fsw/src/lc_cmds.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_cmds.c > CMakeFiles/lc.dir/fsw/src/lc_cmds.c.i

apps/lc/CMakeFiles/lc.dir/fsw/src/lc_cmds.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/lc.dir/fsw/src/lc_cmds.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_cmds.c -o CMakeFiles/lc.dir/fsw/src/lc_cmds.c.s

apps/lc/CMakeFiles/lc.dir/fsw/src/lc_custom.c.o: apps/lc/CMakeFiles/lc.dir/flags.make
apps/lc/CMakeFiles/lc.dir/fsw/src/lc_custom.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_custom.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object apps/lc/CMakeFiles/lc.dir/fsw/src/lc_custom.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/lc.dir/fsw/src/lc_custom.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_custom.c

apps/lc/CMakeFiles/lc.dir/fsw/src/lc_custom.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/lc.dir/fsw/src/lc_custom.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_custom.c > CMakeFiles/lc.dir/fsw/src/lc_custom.c.i

apps/lc/CMakeFiles/lc.dir/fsw/src/lc_custom.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/lc.dir/fsw/src/lc_custom.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_custom.c -o CMakeFiles/lc.dir/fsw/src/lc_custom.c.s

apps/lc/CMakeFiles/lc.dir/fsw/src/lc_watch.c.o: apps/lc/CMakeFiles/lc.dir/flags.make
apps/lc/CMakeFiles/lc.dir/fsw/src/lc_watch.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_watch.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object apps/lc/CMakeFiles/lc.dir/fsw/src/lc_watch.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/lc.dir/fsw/src/lc_watch.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_watch.c

apps/lc/CMakeFiles/lc.dir/fsw/src/lc_watch.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/lc.dir/fsw/src/lc_watch.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_watch.c > CMakeFiles/lc.dir/fsw/src/lc_watch.c.i

apps/lc/CMakeFiles/lc.dir/fsw/src/lc_watch.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/lc.dir/fsw/src/lc_watch.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc/fsw/src/lc_watch.c -o CMakeFiles/lc.dir/fsw/src/lc_watch.c.s

# Object files for target lc
lc_OBJECTS = \
"CMakeFiles/lc.dir/fsw/src/lc_action.c.o" \
"CMakeFiles/lc.dir/fsw/src/lc_app.c.o" \
"CMakeFiles/lc.dir/fsw/src/lc_cmds.c.o" \
"CMakeFiles/lc.dir/fsw/src/lc_custom.c.o" \
"CMakeFiles/lc.dir/fsw/src/lc_watch.c.o"

# External object files for target lc
lc_EXTERNAL_OBJECTS =

apps/lc/lc.so: apps/lc/CMakeFiles/lc.dir/fsw/src/lc_action.c.o
apps/lc/lc.so: apps/lc/CMakeFiles/lc.dir/fsw/src/lc_app.c.o
apps/lc/lc.so: apps/lc/CMakeFiles/lc.dir/fsw/src/lc_cmds.c.o
apps/lc/lc.so: apps/lc/CMakeFiles/lc.dir/fsw/src/lc_custom.c.o
apps/lc/lc.so: apps/lc/CMakeFiles/lc.dir/fsw/src/lc_watch.c.o
apps/lc/lc.so: apps/lc/CMakeFiles/lc.dir/build.make
apps/lc/lc.so: apps/lc/CMakeFiles/lc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking C shared module lc.so"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/lc/CMakeFiles/lc.dir/build: apps/lc/lc.so

.PHONY : apps/lc/CMakeFiles/lc.dir/build

apps/lc/CMakeFiles/lc.dir/clean:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc && $(CMAKE_COMMAND) -P CMakeFiles/lc.dir/cmake_clean.cmake
.PHONY : apps/lc/CMakeFiles/lc.dir/clean

apps/lc/CMakeFiles/lc.dir/depend:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/cfe /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/lc /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/lc/CMakeFiles/lc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/lc/CMakeFiles/lc.dir/depend

