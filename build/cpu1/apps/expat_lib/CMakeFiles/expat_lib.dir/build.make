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
include apps/expat_lib/CMakeFiles/expat_lib.dir/depend.make

# Include the progress variables for this target.
include apps/expat_lib/CMakeFiles/expat_lib.dir/progress.make

# Include the compile flags for this target's objects.
include apps/expat_lib/CMakeFiles/expat_lib.dir/flags.make

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/expat_init.c.o: apps/expat_lib/CMakeFiles/expat_lib.dir/flags.make
apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/expat_init.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/expat_init.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/expat_init.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/expat_lib.dir/fsw/src/expat_init.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/expat_init.c

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/expat_init.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/expat_lib.dir/fsw/src/expat_init.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/expat_init.c > CMakeFiles/expat_lib.dir/fsw/src/expat_init.c.i

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/expat_init.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/expat_lib.dir/fsw/src/expat_init.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/expat_init.c -o CMakeFiles/expat_lib.dir/fsw/src/expat_init.c.s

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmlparse.c.o: apps/expat_lib/CMakeFiles/expat_lib.dir/flags.make
apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmlparse.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmlparse.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmlparse.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/expat_lib.dir/fsw/src/xmlparse.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmlparse.c

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmlparse.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/expat_lib.dir/fsw/src/xmlparse.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmlparse.c > CMakeFiles/expat_lib.dir/fsw/src/xmlparse.c.i

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmlparse.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/expat_lib.dir/fsw/src/xmlparse.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmlparse.c -o CMakeFiles/expat_lib.dir/fsw/src/xmlparse.c.s

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmlrole.c.o: apps/expat_lib/CMakeFiles/expat_lib.dir/flags.make
apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmlrole.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmlrole.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmlrole.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/expat_lib.dir/fsw/src/xmlrole.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmlrole.c

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmlrole.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/expat_lib.dir/fsw/src/xmlrole.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmlrole.c > CMakeFiles/expat_lib.dir/fsw/src/xmlrole.c.i

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmlrole.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/expat_lib.dir/fsw/src/xmlrole.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmlrole.c -o CMakeFiles/expat_lib.dir/fsw/src/xmlrole.c.s

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok.c.o: apps/expat_lib/CMakeFiles/expat_lib.dir/flags.make
apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmltok.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/expat_lib.dir/fsw/src/xmltok.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmltok.c

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/expat_lib.dir/fsw/src/xmltok.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmltok.c > CMakeFiles/expat_lib.dir/fsw/src/xmltok.c.i

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/expat_lib.dir/fsw/src/xmltok.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmltok.c -o CMakeFiles/expat_lib.dir/fsw/src/xmltok.c.s

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok_impl.c.o: apps/expat_lib/CMakeFiles/expat_lib.dir/flags.make
apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok_impl.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmltok_impl.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok_impl.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/expat_lib.dir/fsw/src/xmltok_impl.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmltok_impl.c

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok_impl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/expat_lib.dir/fsw/src/xmltok_impl.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmltok_impl.c > CMakeFiles/expat_lib.dir/fsw/src/xmltok_impl.c.i

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok_impl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/expat_lib.dir/fsw/src/xmltok_impl.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmltok_impl.c -o CMakeFiles/expat_lib.dir/fsw/src/xmltok_impl.c.s

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok_ns.c.o: apps/expat_lib/CMakeFiles/expat_lib.dir/flags.make
apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok_ns.c.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmltok_ns.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok_ns.c.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/expat_lib.dir/fsw/src/xmltok_ns.c.o   -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmltok_ns.c

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok_ns.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/expat_lib.dir/fsw/src/xmltok_ns.c.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmltok_ns.c > CMakeFiles/expat_lib.dir/fsw/src/xmltok_ns.c.i

apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok_ns.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/expat_lib.dir/fsw/src/xmltok_ns.c.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib/fsw/src/xmltok_ns.c -o CMakeFiles/expat_lib.dir/fsw/src/xmltok_ns.c.s

# Object files for target expat_lib
expat_lib_OBJECTS = \
"CMakeFiles/expat_lib.dir/fsw/src/expat_init.c.o" \
"CMakeFiles/expat_lib.dir/fsw/src/xmlparse.c.o" \
"CMakeFiles/expat_lib.dir/fsw/src/xmlrole.c.o" \
"CMakeFiles/expat_lib.dir/fsw/src/xmltok.c.o" \
"CMakeFiles/expat_lib.dir/fsw/src/xmltok_impl.c.o" \
"CMakeFiles/expat_lib.dir/fsw/src/xmltok_ns.c.o"

# External object files for target expat_lib
expat_lib_EXTERNAL_OBJECTS =

apps/expat_lib/expat_lib.so: apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/expat_init.c.o
apps/expat_lib/expat_lib.so: apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmlparse.c.o
apps/expat_lib/expat_lib.so: apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmlrole.c.o
apps/expat_lib/expat_lib.so: apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok.c.o
apps/expat_lib/expat_lib.so: apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok_impl.c.o
apps/expat_lib/expat_lib.so: apps/expat_lib/CMakeFiles/expat_lib.dir/fsw/src/xmltok_ns.c.o
apps/expat_lib/expat_lib.so: apps/expat_lib/CMakeFiles/expat_lib.dir/build.make
apps/expat_lib/expat_lib.so: apps/expat_lib/CMakeFiles/expat_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking C shared module expat_lib.so"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/expat_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/expat_lib/CMakeFiles/expat_lib.dir/build: apps/expat_lib/expat_lib.so

.PHONY : apps/expat_lib/CMakeFiles/expat_lib.dir/build

apps/expat_lib/CMakeFiles/expat_lib.dir/clean:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib && $(CMAKE_COMMAND) -P CMakeFiles/expat_lib.dir/cmake_clean.cmake
.PHONY : apps/expat_lib/CMakeFiles/expat_lib.dir/clean

apps/expat_lib/CMakeFiles/expat_lib.dir/depend:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/cfe /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/expat_lib /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/expat_lib/CMakeFiles/expat_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/expat_lib/CMakeFiles/expat_lib.dir/depend

