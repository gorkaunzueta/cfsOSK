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
include apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/depend.make

# Include the progress variables for this target.
include apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/progress.make

# Include the compile flags for this target's objects.
include apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/flags.make

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/App.cpp.o: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/flags.make
apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/App.cpp.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/App.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/App.cpp.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/osk_cpp_fw.dir/fsw/src/App.cpp.o -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/App.cpp

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/App.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/osk_cpp_fw.dir/fsw/src/App.cpp.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/App.cpp > CMakeFiles/osk_cpp_fw.dir/fsw/src/App.cpp.i

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/App.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/osk_cpp_fw.dir/fsw/src/App.cpp.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/App.cpp -o CMakeFiles/osk_cpp_fw.dir/fsw/src/App.cpp.s

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/AppObj.cpp.o: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/flags.make
apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/AppObj.cpp.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/AppObj.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/AppObj.cpp.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/osk_cpp_fw.dir/fsw/src/AppObj.cpp.o -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/AppObj.cpp

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/AppObj.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/osk_cpp_fw.dir/fsw/src/AppObj.cpp.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/AppObj.cpp > CMakeFiles/osk_cpp_fw.dir/fsw/src/AppObj.cpp.i

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/AppObj.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/osk_cpp_fw.dir/fsw/src/AppObj.cpp.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/AppObj.cpp -o CMakeFiles/osk_cpp_fw.dir/fsw/src/AppObj.cpp.s

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMgr.cpp.o: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/flags.make
apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMgr.cpp.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/CmdMgr.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMgr.cpp.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMgr.cpp.o -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/CmdMgr.cpp

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMgr.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMgr.cpp.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/CmdMgr.cpp > CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMgr.cpp.i

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMgr.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMgr.cpp.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/CmdMgr.cpp -o CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMgr.cpp.s

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMsg.cpp.o: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/flags.make
apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMsg.cpp.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/CmdMsg.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMsg.cpp.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMsg.cpp.o -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/CmdMsg.cpp

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMsg.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMsg.cpp.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/CmdMsg.cpp > CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMsg.cpp.i

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMsg.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMsg.cpp.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/CmdMsg.cpp -o CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMsg.cpp.s

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/FileUtil.cpp.o: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/flags.make
apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/FileUtil.cpp.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/FileUtil.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/FileUtil.cpp.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/osk_cpp_fw.dir/fsw/src/FileUtil.cpp.o -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/FileUtil.cpp

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/FileUtil.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/osk_cpp_fw.dir/fsw/src/FileUtil.cpp.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/FileUtil.cpp > CMakeFiles/osk_cpp_fw.dir/fsw/src/FileUtil.cpp.i

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/FileUtil.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/osk_cpp_fw.dir/fsw/src/FileUtil.cpp.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/FileUtil.cpp -o CMakeFiles/osk_cpp_fw.dir/fsw/src/FileUtil.cpp.s

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Jsmn.cpp.o: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/flags.make
apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Jsmn.cpp.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/Jsmn.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Jsmn.cpp.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/osk_cpp_fw.dir/fsw/src/Jsmn.cpp.o -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/Jsmn.cpp

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Jsmn.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/osk_cpp_fw.dir/fsw/src/Jsmn.cpp.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/Jsmn.cpp > CMakeFiles/osk_cpp_fw.dir/fsw/src/Jsmn.cpp.i

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Jsmn.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/osk_cpp_fw.dir/fsw/src/Jsmn.cpp.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/Jsmn.cpp -o CMakeFiles/osk_cpp_fw.dir/fsw/src/Jsmn.cpp.s

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Json.cpp.o: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/flags.make
apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Json.cpp.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/Json.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Json.cpp.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/osk_cpp_fw.dir/fsw/src/Json.cpp.o -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/Json.cpp

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Json.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/osk_cpp_fw.dir/fsw/src/Json.cpp.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/Json.cpp > CMakeFiles/osk_cpp_fw.dir/fsw/src/Json.cpp.i

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Json.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/osk_cpp_fw.dir/fsw/src/Json.cpp.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/Json.cpp -o CMakeFiles/osk_cpp_fw.dir/fsw/src/Json.cpp.s

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/JsonTblObj.cpp.o: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/flags.make
apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/JsonTblObj.cpp.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/JsonTblObj.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/JsonTblObj.cpp.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/osk_cpp_fw.dir/fsw/src/JsonTblObj.cpp.o -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/JsonTblObj.cpp

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/JsonTblObj.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/osk_cpp_fw.dir/fsw/src/JsonTblObj.cpp.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/JsonTblObj.cpp > CMakeFiles/osk_cpp_fw.dir/fsw/src/JsonTblObj.cpp.i

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/JsonTblObj.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/osk_cpp_fw.dir/fsw/src/JsonTblObj.cpp.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/JsonTblObj.cpp -o CMakeFiles/osk_cpp_fw.dir/fsw/src/JsonTblObj.cpp.s

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/OskCppFwInit.cpp.o: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/flags.make
apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/OskCppFwInit.cpp.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/OskCppFwInit.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/OskCppFwInit.cpp.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/osk_cpp_fw.dir/fsw/src/OskCppFwInit.cpp.o -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/OskCppFwInit.cpp

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/OskCppFwInit.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/osk_cpp_fw.dir/fsw/src/OskCppFwInit.cpp.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/OskCppFwInit.cpp > CMakeFiles/osk_cpp_fw.dir/fsw/src/OskCppFwInit.cpp.i

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/OskCppFwInit.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/osk_cpp_fw.dir/fsw/src/OskCppFwInit.cpp.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/OskCppFwInit.cpp -o CMakeFiles/osk_cpp_fw.dir/fsw/src/OskCppFwInit.cpp.s

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Tbl.cpp.o: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/flags.make
apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Tbl.cpp.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/Tbl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Tbl.cpp.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/osk_cpp_fw.dir/fsw/src/Tbl.cpp.o -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/Tbl.cpp

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Tbl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/osk_cpp_fw.dir/fsw/src/Tbl.cpp.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/Tbl.cpp > CMakeFiles/osk_cpp_fw.dir/fsw/src/Tbl.cpp.i

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Tbl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/osk_cpp_fw.dir/fsw/src/Tbl.cpp.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/Tbl.cpp -o CMakeFiles/osk_cpp_fw.dir/fsw/src/Tbl.cpp.s

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/TblMgr.cpp.o: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/flags.make
apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/TblMgr.cpp.o: /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/TblMgr.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/TblMgr.cpp.o"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/osk_cpp_fw.dir/fsw/src/TblMgr.cpp.o -c /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/TblMgr.cpp

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/TblMgr.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/osk_cpp_fw.dir/fsw/src/TblMgr.cpp.i"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/TblMgr.cpp > CMakeFiles/osk_cpp_fw.dir/fsw/src/TblMgr.cpp.i

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/TblMgr.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/osk_cpp_fw.dir/fsw/src/TblMgr.cpp.s"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw/fsw/src/TblMgr.cpp -o CMakeFiles/osk_cpp_fw.dir/fsw/src/TblMgr.cpp.s

# Object files for target osk_cpp_fw
osk_cpp_fw_OBJECTS = \
"CMakeFiles/osk_cpp_fw.dir/fsw/src/App.cpp.o" \
"CMakeFiles/osk_cpp_fw.dir/fsw/src/AppObj.cpp.o" \
"CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMgr.cpp.o" \
"CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMsg.cpp.o" \
"CMakeFiles/osk_cpp_fw.dir/fsw/src/FileUtil.cpp.o" \
"CMakeFiles/osk_cpp_fw.dir/fsw/src/Jsmn.cpp.o" \
"CMakeFiles/osk_cpp_fw.dir/fsw/src/Json.cpp.o" \
"CMakeFiles/osk_cpp_fw.dir/fsw/src/JsonTblObj.cpp.o" \
"CMakeFiles/osk_cpp_fw.dir/fsw/src/OskCppFwInit.cpp.o" \
"CMakeFiles/osk_cpp_fw.dir/fsw/src/Tbl.cpp.o" \
"CMakeFiles/osk_cpp_fw.dir/fsw/src/TblMgr.cpp.o"

# External object files for target osk_cpp_fw
osk_cpp_fw_EXTERNAL_OBJECTS =

apps/osk_cpp_fw/osk_cpp_fw.so: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/App.cpp.o
apps/osk_cpp_fw/osk_cpp_fw.so: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/AppObj.cpp.o
apps/osk_cpp_fw/osk_cpp_fw.so: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMgr.cpp.o
apps/osk_cpp_fw/osk_cpp_fw.so: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/CmdMsg.cpp.o
apps/osk_cpp_fw/osk_cpp_fw.so: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/FileUtil.cpp.o
apps/osk_cpp_fw/osk_cpp_fw.so: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Jsmn.cpp.o
apps/osk_cpp_fw/osk_cpp_fw.so: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Json.cpp.o
apps/osk_cpp_fw/osk_cpp_fw.so: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/JsonTblObj.cpp.o
apps/osk_cpp_fw/osk_cpp_fw.so: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/OskCppFwInit.cpp.o
apps/osk_cpp_fw/osk_cpp_fw.so: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/Tbl.cpp.o
apps/osk_cpp_fw/osk_cpp_fw.so: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/fsw/src/TblMgr.cpp.o
apps/osk_cpp_fw/osk_cpp_fw.so: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/build.make
apps/osk_cpp_fw/osk_cpp_fw.so: apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Linking CXX shared module osk_cpp_fw.so"
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/osk_cpp_fw.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/build: apps/osk_cpp_fw/osk_cpp_fw.so

.PHONY : apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/build

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/clean:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw && $(CMAKE_COMMAND) -P CMakeFiles/osk_cpp_fw.dir/cmake_clean.cmake
.PHONY : apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/clean

apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/depend:
	cd /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/cfe /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/apps/osk_cpp_fw /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1 /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw /home/pi/plathon/cfs-OSK/OpenSatKit/cfs/build/cpu1/apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/osk_cpp_fw/CMakeFiles/osk_cpp_fw.dir/depend
