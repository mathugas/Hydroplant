# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.24

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/chamat/Hydroplant/project/hydroplant

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chamat/Hydroplant/project/hydroplant/build

# Utility rule file for gen_project_binary.

# Include any custom commands dependencies for this target.
include CMakeFiles/gen_project_binary.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/gen_project_binary.dir/progress.make

CMakeFiles/gen_project_binary: .bin_timestamp

.bin_timestamp: hydroplant.elf
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/chamat/Hydroplant/project/hydroplant/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating binary image from built executable"
	/home/chamat/.espressif/python_env/idf5.1_py3.8_env/bin/python /home/chamat/Hydroplant/esp_rtos/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 elf2image --flash_mode dio --flash_freq 40m --flash_size 2MB --elf-sha256-offset 0xb0 -o /home/chamat/Hydroplant/project/hydroplant/build/hydroplant.bin /home/chamat/Hydroplant/project/hydroplant/build/hydroplant.elf
	/usr/bin/cmake -E echo "Generated /home/chamat/Hydroplant/project/hydroplant/build/hydroplant.bin"
	/usr/bin/cmake -E md5sum /home/chamat/Hydroplant/project/hydroplant/build/hydroplant.bin > /home/chamat/Hydroplant/project/hydroplant/build/.bin_timestamp

gen_project_binary: .bin_timestamp
gen_project_binary: CMakeFiles/gen_project_binary
gen_project_binary: CMakeFiles/gen_project_binary.dir/build.make
.PHONY : gen_project_binary

# Rule to build all files generated by this target.
CMakeFiles/gen_project_binary.dir/build: gen_project_binary
.PHONY : CMakeFiles/gen_project_binary.dir/build

CMakeFiles/gen_project_binary.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gen_project_binary.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gen_project_binary.dir/clean

CMakeFiles/gen_project_binary.dir/depend:
	cd /home/chamat/Hydroplant/project/hydroplant/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chamat/Hydroplant/project/hydroplant /home/chamat/Hydroplant/project/hydroplant /home/chamat/Hydroplant/project/hydroplant/build /home/chamat/Hydroplant/project/hydroplant/build /home/chamat/Hydroplant/project/hydroplant/build/CMakeFiles/gen_project_binary.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gen_project_binary.dir/depend
