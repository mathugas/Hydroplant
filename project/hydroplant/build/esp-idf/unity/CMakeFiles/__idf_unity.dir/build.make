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

# Include any dependencies generated for this target.
include esp-idf/unity/CMakeFiles/__idf_unity.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include esp-idf/unity/CMakeFiles/__idf_unity.dir/compiler_depend.make

# Include the progress variables for this target.
include esp-idf/unity/CMakeFiles/__idf_unity.dir/progress.make

# Include the compile flags for this target's objects.
include esp-idf/unity/CMakeFiles/__idf_unity.dir/flags.make

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity/src/unity.c.obj: esp-idf/unity/CMakeFiles/__idf_unity.dir/flags.make
esp-idf/unity/CMakeFiles/__idf_unity.dir/unity/src/unity.c.obj: /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity/src/unity.c
esp-idf/unity/CMakeFiles/__idf_unity.dir/unity/src/unity.c.obj: esp-idf/unity/CMakeFiles/__idf_unity.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chamat/Hydroplant/project/hydroplant/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object esp-idf/unity/CMakeFiles/__idf_unity.dir/unity/src/unity.c.obj"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/unity/CMakeFiles/__idf_unity.dir/unity/src/unity.c.obj -MF CMakeFiles/__idf_unity.dir/unity/src/unity.c.obj.d -o CMakeFiles/__idf_unity.dir/unity/src/unity.c.obj -c /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity/src/unity.c

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity/src/unity.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_unity.dir/unity/src/unity.c.i"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity/src/unity.c > CMakeFiles/__idf_unity.dir/unity/src/unity.c.i

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity/src/unity.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_unity.dir/unity/src/unity.c.s"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity/src/unity.c -o CMakeFiles/__idf_unity.dir/unity/src/unity.c.s

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_runner.c.obj: esp-idf/unity/CMakeFiles/__idf_unity.dir/flags.make
esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_runner.c.obj: /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_runner.c
esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_runner.c.obj: esp-idf/unity/CMakeFiles/__idf_unity.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chamat/Hydroplant/project/hydroplant/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_runner.c.obj"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_runner.c.obj -MF CMakeFiles/__idf_unity.dir/unity_runner.c.obj.d -o CMakeFiles/__idf_unity.dir/unity_runner.c.obj -c /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_runner.c

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_runner.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_unity.dir/unity_runner.c.i"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_runner.c > CMakeFiles/__idf_unity.dir/unity_runner.c.i

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_runner.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_unity.dir/unity_runner.c.s"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_runner.c -o CMakeFiles/__idf_unity.dir/unity_runner.c.s

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_freertos.c.obj: esp-idf/unity/CMakeFiles/__idf_unity.dir/flags.make
esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_freertos.c.obj: /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_utils_freertos.c
esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_freertos.c.obj: esp-idf/unity/CMakeFiles/__idf_unity.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chamat/Hydroplant/project/hydroplant/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_freertos.c.obj"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_freertos.c.obj -MF CMakeFiles/__idf_unity.dir/unity_utils_freertos.c.obj.d -o CMakeFiles/__idf_unity.dir/unity_utils_freertos.c.obj -c /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_utils_freertos.c

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_freertos.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_unity.dir/unity_utils_freertos.c.i"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_utils_freertos.c > CMakeFiles/__idf_unity.dir/unity_utils_freertos.c.i

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_freertos.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_unity.dir/unity_utils_freertos.c.s"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_utils_freertos.c -o CMakeFiles/__idf_unity.dir/unity_utils_freertos.c.s

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_cache.c.obj: esp-idf/unity/CMakeFiles/__idf_unity.dir/flags.make
esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_cache.c.obj: /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_utils_cache.c
esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_cache.c.obj: esp-idf/unity/CMakeFiles/__idf_unity.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chamat/Hydroplant/project/hydroplant/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_cache.c.obj"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_cache.c.obj -MF CMakeFiles/__idf_unity.dir/unity_utils_cache.c.obj.d -o CMakeFiles/__idf_unity.dir/unity_utils_cache.c.obj -c /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_utils_cache.c

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_cache.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_unity.dir/unity_utils_cache.c.i"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_utils_cache.c > CMakeFiles/__idf_unity.dir/unity_utils_cache.c.i

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_cache.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_unity.dir/unity_utils_cache.c.s"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_utils_cache.c -o CMakeFiles/__idf_unity.dir/unity_utils_cache.c.s

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_memory.c.obj: esp-idf/unity/CMakeFiles/__idf_unity.dir/flags.make
esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_memory.c.obj: /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_utils_memory.c
esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_memory.c.obj: esp-idf/unity/CMakeFiles/__idf_unity.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chamat/Hydroplant/project/hydroplant/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_memory.c.obj"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_memory.c.obj -MF CMakeFiles/__idf_unity.dir/unity_utils_memory.c.obj.d -o CMakeFiles/__idf_unity.dir/unity_utils_memory.c.obj -c /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_utils_memory.c

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_memory.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_unity.dir/unity_utils_memory.c.i"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_utils_memory.c > CMakeFiles/__idf_unity.dir/unity_utils_memory.c.i

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_memory.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_unity.dir/unity_utils_memory.c.s"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_utils_memory.c -o CMakeFiles/__idf_unity.dir/unity_utils_memory.c.s

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_port_esp32.c.obj: esp-idf/unity/CMakeFiles/__idf_unity.dir/flags.make
esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_port_esp32.c.obj: /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_port_esp32.c
esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_port_esp32.c.obj: esp-idf/unity/CMakeFiles/__idf_unity.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chamat/Hydroplant/project/hydroplant/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_port_esp32.c.obj"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_port_esp32.c.obj -MF CMakeFiles/__idf_unity.dir/unity_port_esp32.c.obj.d -o CMakeFiles/__idf_unity.dir/unity_port_esp32.c.obj -c /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_port_esp32.c

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_port_esp32.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_unity.dir/unity_port_esp32.c.i"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_port_esp32.c > CMakeFiles/__idf_unity.dir/unity_port_esp32.c.i

esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_port_esp32.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_unity.dir/unity_port_esp32.c.s"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/unity_port_esp32.c -o CMakeFiles/__idf_unity.dir/unity_port_esp32.c.s

esp-idf/unity/CMakeFiles/__idf_unity.dir/port/esp/unity_utils_memory_esp.c.obj: esp-idf/unity/CMakeFiles/__idf_unity.dir/flags.make
esp-idf/unity/CMakeFiles/__idf_unity.dir/port/esp/unity_utils_memory_esp.c.obj: /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/port/esp/unity_utils_memory_esp.c
esp-idf/unity/CMakeFiles/__idf_unity.dir/port/esp/unity_utils_memory_esp.c.obj: esp-idf/unity/CMakeFiles/__idf_unity.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chamat/Hydroplant/project/hydroplant/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object esp-idf/unity/CMakeFiles/__idf_unity.dir/port/esp/unity_utils_memory_esp.c.obj"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/unity/CMakeFiles/__idf_unity.dir/port/esp/unity_utils_memory_esp.c.obj -MF CMakeFiles/__idf_unity.dir/port/esp/unity_utils_memory_esp.c.obj.d -o CMakeFiles/__idf_unity.dir/port/esp/unity_utils_memory_esp.c.obj -c /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/port/esp/unity_utils_memory_esp.c

esp-idf/unity/CMakeFiles/__idf_unity.dir/port/esp/unity_utils_memory_esp.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_unity.dir/port/esp/unity_utils_memory_esp.c.i"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/port/esp/unity_utils_memory_esp.c > CMakeFiles/__idf_unity.dir/port/esp/unity_utils_memory_esp.c.i

esp-idf/unity/CMakeFiles/__idf_unity.dir/port/esp/unity_utils_memory_esp.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_unity.dir/port/esp/unity_utils_memory_esp.c.s"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity/port/esp/unity_utils_memory_esp.c -o CMakeFiles/__idf_unity.dir/port/esp/unity_utils_memory_esp.c.s

# Object files for target __idf_unity
__idf_unity_OBJECTS = \
"CMakeFiles/__idf_unity.dir/unity/src/unity.c.obj" \
"CMakeFiles/__idf_unity.dir/unity_runner.c.obj" \
"CMakeFiles/__idf_unity.dir/unity_utils_freertos.c.obj" \
"CMakeFiles/__idf_unity.dir/unity_utils_cache.c.obj" \
"CMakeFiles/__idf_unity.dir/unity_utils_memory.c.obj" \
"CMakeFiles/__idf_unity.dir/unity_port_esp32.c.obj" \
"CMakeFiles/__idf_unity.dir/port/esp/unity_utils_memory_esp.c.obj"

# External object files for target __idf_unity
__idf_unity_EXTERNAL_OBJECTS =

esp-idf/unity/libunity.a: esp-idf/unity/CMakeFiles/__idf_unity.dir/unity/src/unity.c.obj
esp-idf/unity/libunity.a: esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_runner.c.obj
esp-idf/unity/libunity.a: esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_freertos.c.obj
esp-idf/unity/libunity.a: esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_cache.c.obj
esp-idf/unity/libunity.a: esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_utils_memory.c.obj
esp-idf/unity/libunity.a: esp-idf/unity/CMakeFiles/__idf_unity.dir/unity_port_esp32.c.obj
esp-idf/unity/libunity.a: esp-idf/unity/CMakeFiles/__idf_unity.dir/port/esp/unity_utils_memory_esp.c.obj
esp-idf/unity/libunity.a: esp-idf/unity/CMakeFiles/__idf_unity.dir/build.make
esp-idf/unity/libunity.a: esp-idf/unity/CMakeFiles/__idf_unity.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chamat/Hydroplant/project/hydroplant/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking C static library libunity.a"
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && $(CMAKE_COMMAND) -P CMakeFiles/__idf_unity.dir/cmake_clean_target.cmake
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/__idf_unity.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp-idf/unity/CMakeFiles/__idf_unity.dir/build: esp-idf/unity/libunity.a
.PHONY : esp-idf/unity/CMakeFiles/__idf_unity.dir/build

esp-idf/unity/CMakeFiles/__idf_unity.dir/clean:
	cd /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity && $(CMAKE_COMMAND) -P CMakeFiles/__idf_unity.dir/cmake_clean.cmake
.PHONY : esp-idf/unity/CMakeFiles/__idf_unity.dir/clean

esp-idf/unity/CMakeFiles/__idf_unity.dir/depend:
	cd /home/chamat/Hydroplant/project/hydroplant/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chamat/Hydroplant/project/hydroplant /home/chamat/Hydroplant/esp_rtos/esp-idf/components/unity /home/chamat/Hydroplant/project/hydroplant/build /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity /home/chamat/Hydroplant/project/hydroplant/build/esp-idf/unity/CMakeFiles/__idf_unity.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/unity/CMakeFiles/__idf_unity.dir/depend

