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
include apps/mm/CMakeFiles/mm.dir/depend.make

# Include the progress variables for this target.
include apps/mm/CMakeFiles/mm.dir/progress.make

# Include the compile flags for this target's objects.
include apps/mm/CMakeFiles/mm.dir/flags.make

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_app.c.o: apps/mm/CMakeFiles/mm.dir/flags.make
apps/mm/CMakeFiles/mm.dir/fsw/src/mm_app.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_app.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object apps/mm/CMakeFiles/mm.dir/fsw/src/mm_app.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/mm.dir/fsw/src/mm_app.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_app.c

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_app.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mm.dir/fsw/src/mm_app.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_app.c > CMakeFiles/mm.dir/fsw/src/mm_app.c.i

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_app.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mm.dir/fsw/src/mm_app.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_app.c -o CMakeFiles/mm.dir/fsw/src/mm_app.c.s

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_dump.c.o: apps/mm/CMakeFiles/mm.dir/flags.make
apps/mm/CMakeFiles/mm.dir/fsw/src/mm_dump.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_dump.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object apps/mm/CMakeFiles/mm.dir/fsw/src/mm_dump.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/mm.dir/fsw/src/mm_dump.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_dump.c

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_dump.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mm.dir/fsw/src/mm_dump.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_dump.c > CMakeFiles/mm.dir/fsw/src/mm_dump.c.i

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_dump.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mm.dir/fsw/src/mm_dump.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_dump.c -o CMakeFiles/mm.dir/fsw/src/mm_dump.c.s

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_load.c.o: apps/mm/CMakeFiles/mm.dir/flags.make
apps/mm/CMakeFiles/mm.dir/fsw/src/mm_load.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_load.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object apps/mm/CMakeFiles/mm.dir/fsw/src/mm_load.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/mm.dir/fsw/src/mm_load.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_load.c

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_load.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mm.dir/fsw/src/mm_load.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_load.c > CMakeFiles/mm.dir/fsw/src/mm_load.c.i

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_load.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mm.dir/fsw/src/mm_load.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_load.c -o CMakeFiles/mm.dir/fsw/src/mm_load.c.s

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem16.c.o: apps/mm/CMakeFiles/mm.dir/flags.make
apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem16.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_mem16.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem16.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/mm.dir/fsw/src/mm_mem16.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_mem16.c

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem16.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mm.dir/fsw/src/mm_mem16.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_mem16.c > CMakeFiles/mm.dir/fsw/src/mm_mem16.c.i

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem16.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mm.dir/fsw/src/mm_mem16.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_mem16.c -o CMakeFiles/mm.dir/fsw/src/mm_mem16.c.s

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem32.c.o: apps/mm/CMakeFiles/mm.dir/flags.make
apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem32.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_mem32.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem32.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/mm.dir/fsw/src/mm_mem32.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_mem32.c

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem32.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mm.dir/fsw/src/mm_mem32.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_mem32.c > CMakeFiles/mm.dir/fsw/src/mm_mem32.c.i

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem32.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mm.dir/fsw/src/mm_mem32.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_mem32.c -o CMakeFiles/mm.dir/fsw/src/mm_mem32.c.s

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem8.c.o: apps/mm/CMakeFiles/mm.dir/flags.make
apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem8.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_mem8.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem8.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/mm.dir/fsw/src/mm_mem8.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_mem8.c

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem8.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mm.dir/fsw/src/mm_mem8.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_mem8.c > CMakeFiles/mm.dir/fsw/src/mm_mem8.c.i

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem8.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mm.dir/fsw/src/mm_mem8.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_mem8.c -o CMakeFiles/mm.dir/fsw/src/mm_mem8.c.s

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_utils.c.o: apps/mm/CMakeFiles/mm.dir/flags.make
apps/mm/CMakeFiles/mm.dir/fsw/src/mm_utils.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_utils.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object apps/mm/CMakeFiles/mm.dir/fsw/src/mm_utils.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/mm.dir/fsw/src/mm_utils.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_utils.c

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_utils.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mm.dir/fsw/src/mm_utils.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_utils.c > CMakeFiles/mm.dir/fsw/src/mm_utils.c.i

apps/mm/CMakeFiles/mm.dir/fsw/src/mm_utils.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mm.dir/fsw/src/mm_utils.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm/fsw/src/mm_utils.c -o CMakeFiles/mm.dir/fsw/src/mm_utils.c.s

# Object files for target mm
mm_OBJECTS = \
"CMakeFiles/mm.dir/fsw/src/mm_app.c.o" \
"CMakeFiles/mm.dir/fsw/src/mm_dump.c.o" \
"CMakeFiles/mm.dir/fsw/src/mm_load.c.o" \
"CMakeFiles/mm.dir/fsw/src/mm_mem16.c.o" \
"CMakeFiles/mm.dir/fsw/src/mm_mem32.c.o" \
"CMakeFiles/mm.dir/fsw/src/mm_mem8.c.o" \
"CMakeFiles/mm.dir/fsw/src/mm_utils.c.o"

# External object files for target mm
mm_EXTERNAL_OBJECTS =

apps/mm/mm.so: apps/mm/CMakeFiles/mm.dir/fsw/src/mm_app.c.o
apps/mm/mm.so: apps/mm/CMakeFiles/mm.dir/fsw/src/mm_dump.c.o
apps/mm/mm.so: apps/mm/CMakeFiles/mm.dir/fsw/src/mm_load.c.o
apps/mm/mm.so: apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem16.c.o
apps/mm/mm.so: apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem32.c.o
apps/mm/mm.so: apps/mm/CMakeFiles/mm.dir/fsw/src/mm_mem8.c.o
apps/mm/mm.so: apps/mm/CMakeFiles/mm.dir/fsw/src/mm_utils.c.o
apps/mm/mm.so: apps/mm/CMakeFiles/mm.dir/build.make
apps/mm/mm.so: apps/mm/CMakeFiles/mm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking C shared module mm.so"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/mm/CMakeFiles/mm.dir/build: apps/mm/mm.so

.PHONY : apps/mm/CMakeFiles/mm.dir/build

apps/mm/CMakeFiles/mm.dir/clean:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm && $(CMAKE_COMMAND) -P CMakeFiles/mm.dir/cmake_clean.cmake
.PHONY : apps/mm/CMakeFiles/mm.dir/clean

apps/mm/CMakeFiles/mm.dir/depend:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/cfe /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/mm /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/mm/CMakeFiles/mm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/mm/CMakeFiles/mm.dir/depend

