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
include apps/fm/CMakeFiles/fm.dir/depend.make

# Include the progress variables for this target.
include apps/fm/CMakeFiles/fm.dir/progress.make

# Include the compile flags for this target's objects.
include apps/fm/CMakeFiles/fm.dir/flags.make

apps/fm/CMakeFiles/fm.dir/fsw/src/fm_app.c.o: apps/fm/CMakeFiles/fm.dir/flags.make
apps/fm/CMakeFiles/fm.dir/fsw/src/fm_app.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_app.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object apps/fm/CMakeFiles/fm.dir/fsw/src/fm_app.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/fm.dir/fsw/src/fm_app.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_app.c

apps/fm/CMakeFiles/fm.dir/fsw/src/fm_app.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/fm.dir/fsw/src/fm_app.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_app.c > CMakeFiles/fm.dir/fsw/src/fm_app.c.i

apps/fm/CMakeFiles/fm.dir/fsw/src/fm_app.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/fm.dir/fsw/src/fm_app.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_app.c -o CMakeFiles/fm.dir/fsw/src/fm_app.c.s

apps/fm/CMakeFiles/fm.dir/fsw/src/fm_child.c.o: apps/fm/CMakeFiles/fm.dir/flags.make
apps/fm/CMakeFiles/fm.dir/fsw/src/fm_child.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_child.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object apps/fm/CMakeFiles/fm.dir/fsw/src/fm_child.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/fm.dir/fsw/src/fm_child.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_child.c

apps/fm/CMakeFiles/fm.dir/fsw/src/fm_child.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/fm.dir/fsw/src/fm_child.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_child.c > CMakeFiles/fm.dir/fsw/src/fm_child.c.i

apps/fm/CMakeFiles/fm.dir/fsw/src/fm_child.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/fm.dir/fsw/src/fm_child.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_child.c -o CMakeFiles/fm.dir/fsw/src/fm_child.c.s

apps/fm/CMakeFiles/fm.dir/fsw/src/fm_cmd_utils.c.o: apps/fm/CMakeFiles/fm.dir/flags.make
apps/fm/CMakeFiles/fm.dir/fsw/src/fm_cmd_utils.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_cmd_utils.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object apps/fm/CMakeFiles/fm.dir/fsw/src/fm_cmd_utils.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/fm.dir/fsw/src/fm_cmd_utils.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_cmd_utils.c

apps/fm/CMakeFiles/fm.dir/fsw/src/fm_cmd_utils.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/fm.dir/fsw/src/fm_cmd_utils.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_cmd_utils.c > CMakeFiles/fm.dir/fsw/src/fm_cmd_utils.c.i

apps/fm/CMakeFiles/fm.dir/fsw/src/fm_cmd_utils.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/fm.dir/fsw/src/fm_cmd_utils.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_cmd_utils.c -o CMakeFiles/fm.dir/fsw/src/fm_cmd_utils.c.s

apps/fm/CMakeFiles/fm.dir/fsw/src/fm_cmds.c.o: apps/fm/CMakeFiles/fm.dir/flags.make
apps/fm/CMakeFiles/fm.dir/fsw/src/fm_cmds.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_cmds.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object apps/fm/CMakeFiles/fm.dir/fsw/src/fm_cmds.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/fm.dir/fsw/src/fm_cmds.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_cmds.c

apps/fm/CMakeFiles/fm.dir/fsw/src/fm_cmds.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/fm.dir/fsw/src/fm_cmds.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_cmds.c > CMakeFiles/fm.dir/fsw/src/fm_cmds.c.i

apps/fm/CMakeFiles/fm.dir/fsw/src/fm_cmds.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/fm.dir/fsw/src/fm_cmds.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_cmds.c -o CMakeFiles/fm.dir/fsw/src/fm_cmds.c.s

apps/fm/CMakeFiles/fm.dir/fsw/src/fm_tbl.c.o: apps/fm/CMakeFiles/fm.dir/flags.make
apps/fm/CMakeFiles/fm.dir/fsw/src/fm_tbl.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_tbl.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object apps/fm/CMakeFiles/fm.dir/fsw/src/fm_tbl.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/fm.dir/fsw/src/fm_tbl.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_tbl.c

apps/fm/CMakeFiles/fm.dir/fsw/src/fm_tbl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/fm.dir/fsw/src/fm_tbl.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_tbl.c > CMakeFiles/fm.dir/fsw/src/fm_tbl.c.i

apps/fm/CMakeFiles/fm.dir/fsw/src/fm_tbl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/fm.dir/fsw/src/fm_tbl.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm/fsw/src/fm_tbl.c -o CMakeFiles/fm.dir/fsw/src/fm_tbl.c.s

# Object files for target fm
fm_OBJECTS = \
"CMakeFiles/fm.dir/fsw/src/fm_app.c.o" \
"CMakeFiles/fm.dir/fsw/src/fm_child.c.o" \
"CMakeFiles/fm.dir/fsw/src/fm_cmd_utils.c.o" \
"CMakeFiles/fm.dir/fsw/src/fm_cmds.c.o" \
"CMakeFiles/fm.dir/fsw/src/fm_tbl.c.o"

# External object files for target fm
fm_EXTERNAL_OBJECTS =

apps/fm/fm.so: apps/fm/CMakeFiles/fm.dir/fsw/src/fm_app.c.o
apps/fm/fm.so: apps/fm/CMakeFiles/fm.dir/fsw/src/fm_child.c.o
apps/fm/fm.so: apps/fm/CMakeFiles/fm.dir/fsw/src/fm_cmd_utils.c.o
apps/fm/fm.so: apps/fm/CMakeFiles/fm.dir/fsw/src/fm_cmds.c.o
apps/fm/fm.so: apps/fm/CMakeFiles/fm.dir/fsw/src/fm_tbl.c.o
apps/fm/fm.so: apps/fm/CMakeFiles/fm.dir/build.make
apps/fm/fm.so: apps/fm/CMakeFiles/fm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking C shared module fm.so"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/fm/CMakeFiles/fm.dir/build: apps/fm/fm.so

.PHONY : apps/fm/CMakeFiles/fm.dir/build

apps/fm/CMakeFiles/fm.dir/clean:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm && $(CMAKE_COMMAND) -P CMakeFiles/fm.dir/cmake_clean.cmake
.PHONY : apps/fm/CMakeFiles/fm.dir/clean

apps/fm/CMakeFiles/fm.dir/depend:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/cfe /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/fm /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/fm/CMakeFiles/fm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/fm/CMakeFiles/fm.dir/depend

