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
include osal/CMakeFiles/ut_bsp.dir/depend.make

# Include the progress variables for this target.
include osal/CMakeFiles/ut_bsp.dir/progress.make

# Include the compile flags for this target's objects.
include osal/CMakeFiles/ut_bsp.dir/flags.make

osal/CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut.c.o: osal/CMakeFiles/ut_bsp.dir/flags.make
osal/CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osal/src/bsp/pc-linux/ut-src/bsp_ut.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object osal/CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/osal && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osal/src/bsp/pc-linux/ut-src/bsp_ut.c

osal/CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/osal && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osal/src/bsp/pc-linux/ut-src/bsp_ut.c > CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut.c.i

osal/CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/osal && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osal/src/bsp/pc-linux/ut-src/bsp_ut.c -o CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut.c.s

osal/CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut_voltab.c.o: osal/CMakeFiles/ut_bsp.dir/flags.make
osal/CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut_voltab.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osal/src/bsp/pc-linux/ut-src/bsp_ut_voltab.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object osal/CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut_voltab.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/osal && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut_voltab.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osal/src/bsp/pc-linux/ut-src/bsp_ut_voltab.c

osal/CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut_voltab.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut_voltab.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/osal && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osal/src/bsp/pc-linux/ut-src/bsp_ut_voltab.c > CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut_voltab.c.i

osal/CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut_voltab.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut_voltab.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/osal && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osal/src/bsp/pc-linux/ut-src/bsp_ut_voltab.c -o CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut_voltab.c.s

# Object files for target ut_bsp
ut_bsp_OBJECTS = \
"CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut.c.o" \
"CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut_voltab.c.o"

# External object files for target ut_bsp
ut_bsp_EXTERNAL_OBJECTS =

osal/libut_bsp.a: osal/CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut.c.o
osal/libut_bsp.a: osal/CMakeFiles/ut_bsp.dir/src/bsp/pc-linux/ut-src/bsp_ut_voltab.c.o
osal/libut_bsp.a: osal/CMakeFiles/ut_bsp.dir/build.make
osal/libut_bsp.a: osal/CMakeFiles/ut_bsp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C static library libut_bsp.a"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/osal && $(CMAKE_COMMAND) -P CMakeFiles/ut_bsp.dir/cmake_clean_target.cmake
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/osal && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ut_bsp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
osal/CMakeFiles/ut_bsp.dir/build: osal/libut_bsp.a

.PHONY : osal/CMakeFiles/ut_bsp.dir/build

osal/CMakeFiles/ut_bsp.dir/clean:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/osal && $(CMAKE_COMMAND) -P CMakeFiles/ut_bsp.dir/cmake_clean.cmake
.PHONY : osal/CMakeFiles/ut_bsp.dir/clean

osal/CMakeFiles/ut_bsp.dir/depend:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/cfe /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osal /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/osal /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/osal/CMakeFiles/ut_bsp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : osal/CMakeFiles/ut_bsp.dir/depend

