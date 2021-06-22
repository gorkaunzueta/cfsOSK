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

# Utility rule file for hk_tables.

# Include the progress variables for this target.
include apps/hk/CMakeFiles/hk_tables.dir/progress.make

apps/hk/CMakeFiles/hk_tables: apps/hk/tables_cpu1/hk_cpy_tbl.tbl


apps/hk/tables_cpu1/hk_cpy_tbl.tbl: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/tools/elf2cfetbl/elf2cfetbl
apps/hk/tables_cpu1/hk_cpy_tbl.tbl: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/hk/fsw/tables/hk_cpy_tbl.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating tables_cpu1/hk_cpy_tbl.tbl"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/hk/tables_cpu1 && /usr/bin/gcc -Wall -D_XOPEN_SOURCE=600 -D_LINUX_OS_ -I/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/inc -I/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/inc -I/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/osal/src/os/inc -I/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/psp/fsw/inc -I/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/cfe/fsw/cfe-core/src/inc -I/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/cfe/cmake/target/inc -I/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/cfe_core_default_cpu1/inc -I/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/hk/fsw/src -I/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/hk/fsw/mission_inc -I/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/hk/fsw/platform_inc -I/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/cfs_lib/fsw/public_inc -I/fsw/public_inc -I/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/cfe/fsw/cfe-core/src/inc -I/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/f42/fsw/platform_inc -I/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/isim/fsw/platform_inc -c -o hk_cpy_tbl.o /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/hk/fsw/tables/hk_cpy_tbl.c
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/hk/tables_cpu1 && /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/tools/elf2cfetbl/elf2cfetbl hk_cpy_tbl.o

hk_tables: apps/hk/CMakeFiles/hk_tables
hk_tables: apps/hk/tables_cpu1/hk_cpy_tbl.tbl
hk_tables: apps/hk/CMakeFiles/hk_tables.dir/build.make

.PHONY : hk_tables

# Rule to build all files generated by this target.
apps/hk/CMakeFiles/hk_tables.dir/build: hk_tables

.PHONY : apps/hk/CMakeFiles/hk_tables.dir/build

apps/hk/CMakeFiles/hk_tables.dir/clean:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/hk && $(CMAKE_COMMAND) -P CMakeFiles/hk_tables.dir/cmake_clean.cmake
.PHONY : apps/hk/CMakeFiles/hk_tables.dir/clean

apps/hk/CMakeFiles/hk_tables.dir/depend:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/cfe /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/hk /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/hk /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/hk/CMakeFiles/hk_tables.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/hk/CMakeFiles/hk_tables.dir/depend

