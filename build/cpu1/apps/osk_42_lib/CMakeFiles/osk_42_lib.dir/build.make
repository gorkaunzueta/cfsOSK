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
include apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/depend.make

# Include the progress variables for this target.
include apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/progress.make

# Include the compile flags for this target's objects.
include apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/flags.make

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AcApp.c.o: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/flags.make
apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AcApp.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/AcApp.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AcApp.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/osk_42_lib.dir/fsw/src/AcApp.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/AcApp.c

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AcApp.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/osk_42_lib.dir/fsw/src/AcApp.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/AcApp.c > CMakeFiles/osk_42_lib.dir/fsw/src/AcApp.c.i

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AcApp.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/osk_42_lib.dir/fsw/src/AcApp.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/AcApp.c -o CMakeFiles/osk_42_lib.dir/fsw/src/AcApp.c.s

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AppReadFromSocket.c.o: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/flags.make
apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AppReadFromSocket.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/AppReadFromSocket.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AppReadFromSocket.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/osk_42_lib.dir/fsw/src/AppReadFromSocket.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/AppReadFromSocket.c

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AppReadFromSocket.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/osk_42_lib.dir/fsw/src/AppReadFromSocket.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/AppReadFromSocket.c > CMakeFiles/osk_42_lib.dir/fsw/src/AppReadFromSocket.c.i

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AppReadFromSocket.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/osk_42_lib.dir/fsw/src/AppReadFromSocket.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/AppReadFromSocket.c -o CMakeFiles/osk_42_lib.dir/fsw/src/AppReadFromSocket.c.s

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AppWriteToSocket.c.o: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/flags.make
apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AppWriteToSocket.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/AppWriteToSocket.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AppWriteToSocket.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/osk_42_lib.dir/fsw/src/AppWriteToSocket.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/AppWriteToSocket.c

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AppWriteToSocket.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/osk_42_lib.dir/fsw/src/AppWriteToSocket.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/AppWriteToSocket.c > CMakeFiles/osk_42_lib.dir/fsw/src/AppWriteToSocket.c.i

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AppWriteToSocket.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/osk_42_lib.dir/fsw/src/AppWriteToSocket.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/AppWriteToSocket.c -o CMakeFiles/osk_42_lib.dir/fsw/src/AppWriteToSocket.c.s

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/ac42.c.o: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/flags.make
apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/ac42.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/ac42.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/ac42.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/osk_42_lib.dir/fsw/src/ac42.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/ac42.c

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/ac42.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/osk_42_lib.dir/fsw/src/ac42.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/ac42.c > CMakeFiles/osk_42_lib.dir/fsw/src/ac42.c.i

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/ac42.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/osk_42_lib.dir/fsw/src/ac42.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/ac42.c -o CMakeFiles/osk_42_lib.dir/fsw/src/ac42.c.s

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/dcmkit.c.o: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/flags.make
apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/dcmkit.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/dcmkit.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/dcmkit.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/osk_42_lib.dir/fsw/src/dcmkit.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/dcmkit.c

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/dcmkit.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/osk_42_lib.dir/fsw/src/dcmkit.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/dcmkit.c > CMakeFiles/osk_42_lib.dir/fsw/src/dcmkit.c.i

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/dcmkit.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/osk_42_lib.dir/fsw/src/dcmkit.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/dcmkit.c -o CMakeFiles/osk_42_lib.dir/fsw/src/dcmkit.c.s

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/fswkit.c.o: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/flags.make
apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/fswkit.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/fswkit.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/fswkit.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/osk_42_lib.dir/fsw/src/fswkit.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/fswkit.c

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/fswkit.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/osk_42_lib.dir/fsw/src/fswkit.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/fswkit.c > CMakeFiles/osk_42_lib.dir/fsw/src/fswkit.c.i

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/fswkit.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/osk_42_lib.dir/fsw/src/fswkit.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/fswkit.c -o CMakeFiles/osk_42_lib.dir/fsw/src/fswkit.c.s

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/mathkit.c.o: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/flags.make
apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/mathkit.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/mathkit.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/mathkit.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/osk_42_lib.dir/fsw/src/mathkit.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/mathkit.c

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/mathkit.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/osk_42_lib.dir/fsw/src/mathkit.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/mathkit.c > CMakeFiles/osk_42_lib.dir/fsw/src/mathkit.c.i

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/mathkit.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/osk_42_lib.dir/fsw/src/mathkit.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/mathkit.c -o CMakeFiles/osk_42_lib.dir/fsw/src/mathkit.c.s

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/osk_42_lib_init.c.o: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/flags.make
apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/osk_42_lib_init.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/osk_42_lib_init.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/osk_42_lib_init.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/osk_42_lib.dir/fsw/src/osk_42_lib_init.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/osk_42_lib_init.c

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/osk_42_lib_init.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/osk_42_lib.dir/fsw/src/osk_42_lib_init.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/osk_42_lib_init.c > CMakeFiles/osk_42_lib.dir/fsw/src/osk_42_lib_init.c.i

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/osk_42_lib_init.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/osk_42_lib.dir/fsw/src/osk_42_lib_init.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/osk_42_lib_init.c -o CMakeFiles/osk_42_lib.dir/fsw/src/osk_42_lib_init.c.s

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/timekit.c.o: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/flags.make
apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/timekit.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/timekit.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/timekit.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/osk_42_lib.dir/fsw/src/timekit.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/timekit.c

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/timekit.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/osk_42_lib.dir/fsw/src/timekit.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/timekit.c > CMakeFiles/osk_42_lib.dir/fsw/src/timekit.c.i

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/timekit.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/osk_42_lib.dir/fsw/src/timekit.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib/fsw/src/timekit.c -o CMakeFiles/osk_42_lib.dir/fsw/src/timekit.c.s

# Object files for target osk_42_lib
osk_42_lib_OBJECTS = \
"CMakeFiles/osk_42_lib.dir/fsw/src/AcApp.c.o" \
"CMakeFiles/osk_42_lib.dir/fsw/src/AppReadFromSocket.c.o" \
"CMakeFiles/osk_42_lib.dir/fsw/src/AppWriteToSocket.c.o" \
"CMakeFiles/osk_42_lib.dir/fsw/src/ac42.c.o" \
"CMakeFiles/osk_42_lib.dir/fsw/src/dcmkit.c.o" \
"CMakeFiles/osk_42_lib.dir/fsw/src/fswkit.c.o" \
"CMakeFiles/osk_42_lib.dir/fsw/src/mathkit.c.o" \
"CMakeFiles/osk_42_lib.dir/fsw/src/osk_42_lib_init.c.o" \
"CMakeFiles/osk_42_lib.dir/fsw/src/timekit.c.o"

# External object files for target osk_42_lib
osk_42_lib_EXTERNAL_OBJECTS =

apps/osk_42_lib/osk_42_lib.so: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AcApp.c.o
apps/osk_42_lib/osk_42_lib.so: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AppReadFromSocket.c.o
apps/osk_42_lib/osk_42_lib.so: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/AppWriteToSocket.c.o
apps/osk_42_lib/osk_42_lib.so: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/ac42.c.o
apps/osk_42_lib/osk_42_lib.so: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/dcmkit.c.o
apps/osk_42_lib/osk_42_lib.so: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/fswkit.c.o
apps/osk_42_lib/osk_42_lib.so: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/mathkit.c.o
apps/osk_42_lib/osk_42_lib.so: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/osk_42_lib_init.c.o
apps/osk_42_lib/osk_42_lib.so: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/fsw/src/timekit.c.o
apps/osk_42_lib/osk_42_lib.so: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/build.make
apps/osk_42_lib/osk_42_lib.so: apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking C shared module osk_42_lib.so"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/osk_42_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/build: apps/osk_42_lib/osk_42_lib.so

.PHONY : apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/build

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/clean:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib && $(CMAKE_COMMAND) -P CMakeFiles/osk_42_lib.dir/cmake_clean.cmake
.PHONY : apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/clean

apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/depend:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/cfe /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_42_lib /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/osk_42_lib/CMakeFiles/osk_42_lib.dir/depend

