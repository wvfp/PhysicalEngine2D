# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.30

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = D:\OpenGL\glfw-3.4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = D:\OpenGL\glfw-3.4\build

# Include any dependencies generated for this target.
include tests/CMakeFiles/inputlag.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include tests/CMakeFiles/inputlag.dir/compiler_depend.make

# Include the progress variables for this target.
include tests/CMakeFiles/inputlag.dir/progress.make

# Include the compile flags for this target's objects.
include tests/CMakeFiles/inputlag.dir/flags.make

tests/CMakeFiles/inputlag.dir/inputlag.c.obj: tests/CMakeFiles/inputlag.dir/flags.make
tests/CMakeFiles/inputlag.dir/inputlag.c.obj: tests/CMakeFiles/inputlag.dir/includes_C.rsp
tests/CMakeFiles/inputlag.dir/inputlag.c.obj: D:/OpenGL/glfw-3.4/tests/inputlag.c
tests/CMakeFiles/inputlag.dir/inputlag.c.obj: tests/CMakeFiles/inputlag.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\OpenGL\glfw-3.4\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object tests/CMakeFiles/inputlag.dir/inputlag.c.obj"
	cd /d D:\OpenGL\glfw-3.4\build\tests && C:\MinGW\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT tests/CMakeFiles/inputlag.dir/inputlag.c.obj -MF CMakeFiles\inputlag.dir\inputlag.c.obj.d -o CMakeFiles\inputlag.dir\inputlag.c.obj -c D:\OpenGL\glfw-3.4\tests\inputlag.c

tests/CMakeFiles/inputlag.dir/inputlag.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/inputlag.dir/inputlag.c.i"
	cd /d D:\OpenGL\glfw-3.4\build\tests && C:\MinGW\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\OpenGL\glfw-3.4\tests\inputlag.c > CMakeFiles\inputlag.dir\inputlag.c.i

tests/CMakeFiles/inputlag.dir/inputlag.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/inputlag.dir/inputlag.c.s"
	cd /d D:\OpenGL\glfw-3.4\build\tests && C:\MinGW\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\OpenGL\glfw-3.4\tests\inputlag.c -o CMakeFiles\inputlag.dir\inputlag.c.s

tests/CMakeFiles/inputlag.dir/__/deps/getopt.c.obj: tests/CMakeFiles/inputlag.dir/flags.make
tests/CMakeFiles/inputlag.dir/__/deps/getopt.c.obj: tests/CMakeFiles/inputlag.dir/includes_C.rsp
tests/CMakeFiles/inputlag.dir/__/deps/getopt.c.obj: D:/OpenGL/glfw-3.4/deps/getopt.c
tests/CMakeFiles/inputlag.dir/__/deps/getopt.c.obj: tests/CMakeFiles/inputlag.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=D:\OpenGL\glfw-3.4\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object tests/CMakeFiles/inputlag.dir/__/deps/getopt.c.obj"
	cd /d D:\OpenGL\glfw-3.4\build\tests && C:\MinGW\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT tests/CMakeFiles/inputlag.dir/__/deps/getopt.c.obj -MF CMakeFiles\inputlag.dir\__\deps\getopt.c.obj.d -o CMakeFiles\inputlag.dir\__\deps\getopt.c.obj -c D:\OpenGL\glfw-3.4\deps\getopt.c

tests/CMakeFiles/inputlag.dir/__/deps/getopt.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/inputlag.dir/__/deps/getopt.c.i"
	cd /d D:\OpenGL\glfw-3.4\build\tests && C:\MinGW\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E D:\OpenGL\glfw-3.4\deps\getopt.c > CMakeFiles\inputlag.dir\__\deps\getopt.c.i

tests/CMakeFiles/inputlag.dir/__/deps/getopt.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/inputlag.dir/__/deps/getopt.c.s"
	cd /d D:\OpenGL\glfw-3.4\build\tests && C:\MinGW\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S D:\OpenGL\glfw-3.4\deps\getopt.c -o CMakeFiles\inputlag.dir\__\deps\getopt.c.s

# Object files for target inputlag
inputlag_OBJECTS = \
"CMakeFiles/inputlag.dir/inputlag.c.obj" \
"CMakeFiles/inputlag.dir/__/deps/getopt.c.obj"

# External object files for target inputlag
inputlag_EXTERNAL_OBJECTS =

tests/inputlag.exe: tests/CMakeFiles/inputlag.dir/inputlag.c.obj
tests/inputlag.exe: tests/CMakeFiles/inputlag.dir/__/deps/getopt.c.obj
tests/inputlag.exe: tests/CMakeFiles/inputlag.dir/build.make
tests/inputlag.exe: src/libglfw3.a
tests/inputlag.exe: tests/CMakeFiles/inputlag.dir/linkLibs.rsp
tests/inputlag.exe: tests/CMakeFiles/inputlag.dir/objects1.rsp
tests/inputlag.exe: tests/CMakeFiles/inputlag.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=D:\OpenGL\glfw-3.4\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable inputlag.exe"
	cd /d D:\OpenGL\glfw-3.4\build\tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\inputlag.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tests/CMakeFiles/inputlag.dir/build: tests/inputlag.exe
.PHONY : tests/CMakeFiles/inputlag.dir/build

tests/CMakeFiles/inputlag.dir/clean:
	cd /d D:\OpenGL\glfw-3.4\build\tests && $(CMAKE_COMMAND) -P CMakeFiles\inputlag.dir\cmake_clean.cmake
.PHONY : tests/CMakeFiles/inputlag.dir/clean

tests/CMakeFiles/inputlag.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\OpenGL\glfw-3.4 D:\OpenGL\glfw-3.4\tests D:\OpenGL\glfw-3.4\build D:\OpenGL\glfw-3.4\build\tests D:\OpenGL\glfw-3.4\build\tests\CMakeFiles\inputlag.dir\DependInfo.cmake "--color=$(COLOR)"
.PHONY : tests/CMakeFiles/inputlag.dir/depend

