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
include apps/md/CMakeFiles/md.dir/depend.make

# Include the progress variables for this target.
include apps/md/CMakeFiles/md.dir/progress.make

# Include the compile flags for this target's objects.
include apps/md/CMakeFiles/md.dir/flags.make

apps/md/CMakeFiles/md.dir/fsw/src/md_app.c.o: apps/md/CMakeFiles/md.dir/flags.make
apps/md/CMakeFiles/md.dir/fsw/src/md_app.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_app.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object apps/md/CMakeFiles/md.dir/fsw/src/md_app.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/md.dir/fsw/src/md_app.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_app.c

apps/md/CMakeFiles/md.dir/fsw/src/md_app.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/md.dir/fsw/src/md_app.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_app.c > CMakeFiles/md.dir/fsw/src/md_app.c.i

apps/md/CMakeFiles/md.dir/fsw/src/md_app.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/md.dir/fsw/src/md_app.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_app.c -o CMakeFiles/md.dir/fsw/src/md_app.c.s

apps/md/CMakeFiles/md.dir/fsw/src/md_cmds.c.o: apps/md/CMakeFiles/md.dir/flags.make
apps/md/CMakeFiles/md.dir/fsw/src/md_cmds.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_cmds.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object apps/md/CMakeFiles/md.dir/fsw/src/md_cmds.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/md.dir/fsw/src/md_cmds.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_cmds.c

apps/md/CMakeFiles/md.dir/fsw/src/md_cmds.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/md.dir/fsw/src/md_cmds.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_cmds.c > CMakeFiles/md.dir/fsw/src/md_cmds.c.i

apps/md/CMakeFiles/md.dir/fsw/src/md_cmds.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/md.dir/fsw/src/md_cmds.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_cmds.c -o CMakeFiles/md.dir/fsw/src/md_cmds.c.s

apps/md/CMakeFiles/md.dir/fsw/src/md_dwell_pkt.c.o: apps/md/CMakeFiles/md.dir/flags.make
apps/md/CMakeFiles/md.dir/fsw/src/md_dwell_pkt.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_dwell_pkt.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object apps/md/CMakeFiles/md.dir/fsw/src/md_dwell_pkt.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/md.dir/fsw/src/md_dwell_pkt.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_dwell_pkt.c

apps/md/CMakeFiles/md.dir/fsw/src/md_dwell_pkt.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/md.dir/fsw/src/md_dwell_pkt.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_dwell_pkt.c > CMakeFiles/md.dir/fsw/src/md_dwell_pkt.c.i

apps/md/CMakeFiles/md.dir/fsw/src/md_dwell_pkt.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/md.dir/fsw/src/md_dwell_pkt.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_dwell_pkt.c -o CMakeFiles/md.dir/fsw/src/md_dwell_pkt.c.s

apps/md/CMakeFiles/md.dir/fsw/src/md_dwell_tbl.c.o: apps/md/CMakeFiles/md.dir/flags.make
apps/md/CMakeFiles/md.dir/fsw/src/md_dwell_tbl.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_dwell_tbl.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object apps/md/CMakeFiles/md.dir/fsw/src/md_dwell_tbl.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/md.dir/fsw/src/md_dwell_tbl.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_dwell_tbl.c

apps/md/CMakeFiles/md.dir/fsw/src/md_dwell_tbl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/md.dir/fsw/src/md_dwell_tbl.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_dwell_tbl.c > CMakeFiles/md.dir/fsw/src/md_dwell_tbl.c.i

apps/md/CMakeFiles/md.dir/fsw/src/md_dwell_tbl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/md.dir/fsw/src/md_dwell_tbl.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_dwell_tbl.c -o CMakeFiles/md.dir/fsw/src/md_dwell_tbl.c.s

apps/md/CMakeFiles/md.dir/fsw/src/md_utils.c.o: apps/md/CMakeFiles/md.dir/flags.make
apps/md/CMakeFiles/md.dir/fsw/src/md_utils.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_utils.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object apps/md/CMakeFiles/md.dir/fsw/src/md_utils.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/md.dir/fsw/src/md_utils.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_utils.c

apps/md/CMakeFiles/md.dir/fsw/src/md_utils.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/md.dir/fsw/src/md_utils.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_utils.c > CMakeFiles/md.dir/fsw/src/md_utils.c.i

apps/md/CMakeFiles/md.dir/fsw/src/md_utils.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/md.dir/fsw/src/md_utils.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md/fsw/src/md_utils.c -o CMakeFiles/md.dir/fsw/src/md_utils.c.s

# Object files for target md
md_OBJECTS = \
"CMakeFiles/md.dir/fsw/src/md_app.c.o" \
"CMakeFiles/md.dir/fsw/src/md_cmds.c.o" \
"CMakeFiles/md.dir/fsw/src/md_dwell_pkt.c.o" \
"CMakeFiles/md.dir/fsw/src/md_dwell_tbl.c.o" \
"CMakeFiles/md.dir/fsw/src/md_utils.c.o"

# External object files for target md
md_EXTERNAL_OBJECTS =

apps/md/md.so: apps/md/CMakeFiles/md.dir/fsw/src/md_app.c.o
apps/md/md.so: apps/md/CMakeFiles/md.dir/fsw/src/md_cmds.c.o
apps/md/md.so: apps/md/CMakeFiles/md.dir/fsw/src/md_dwell_pkt.c.o
apps/md/md.so: apps/md/CMakeFiles/md.dir/fsw/src/md_dwell_tbl.c.o
apps/md/md.so: apps/md/CMakeFiles/md.dir/fsw/src/md_utils.c.o
apps/md/md.so: apps/md/CMakeFiles/md.dir/build.make
apps/md/md.so: apps/md/CMakeFiles/md.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking C shared module md.so"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/md.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/md/CMakeFiles/md.dir/build: apps/md/md.so

.PHONY : apps/md/CMakeFiles/md.dir/build

apps/md/CMakeFiles/md.dir/clean:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md && $(CMAKE_COMMAND) -P CMakeFiles/md.dir/cmake_clean.cmake
.PHONY : apps/md/CMakeFiles/md.dir/clean

apps/md/CMakeFiles/md.dir/depend:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/cfe /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/md /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/md/CMakeFiles/md.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/md/CMakeFiles/md.dir/depend

