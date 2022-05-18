# Acceleration Skinning


## On Windows system with Visual Studio 

- Use CMakeLists.txt with Visual Studio
- Precompiled version of GLFW3 is provided (precompiled/glfw3_win)

=> You need to copy assets/ and scenes/ directories in the executable directory

=> Run the CopyAssets.bat file to copy these required directories. In case of issues, modify the 'dst' variable within the batch file to the appropriate local path.



## Compilation in command line using the Makefile (Linux/MacOS only)

$ make

$ ./scene


## Setup compilation in command line using CMake (Linux/MacOs)

This step create the build directory, call CMake to generate the Makefile, compile and execute the code. Note that the creation of the build directory and CMake call has to be done only once on a given system.

The following command assume you have opened a command line in the directory vcl/

### Create a build directory

$ mkdir build

$ cd build

### Execute CMake, compile

$ cmake ..

$ make

$ cd ..

### Execute

$ build/pgm


## Integration with Maya
- Prepare animation in Maya
- Load the python scripts from the **maya_scritps** directory
- To export the character rig without animation, simply execute the script **export_rig.py**
- To export the character and it's animation:
	- Select the mesh of the animated object and execute **set_target_mesh.py**
	- Select the root joint of the animated object's skeleton and execute **set_target_skeleton_root.py**
	- Execute the script **export_rig_and_animation.py**
- Copy the generated files into **assets/custom**
