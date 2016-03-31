# 3D Lens Raytracer

This repository holds a raytracing library for simulating and testing camera lens stacks.

![alt text](https://github.com/erik-nelson/lens_raytracer_3d/blob/master/simulation.png "Lens Raytracing Simulation")

## Status
This library is currently in development.

## Overview
This repository is entirely written in C++, and is structured around the CMAKE cross-compilation paradigm. All source code is in the `lens_raytracer_3d/src/cpp/` directory.

## Easy Build and Run Instructions
Open a MATLAB instance, then

```MATLAB
cd lens_raytracer_3d/src/matlab/
example_create_scene_file
```
The `example_create_scene_file` MATLAB script will automatically compile the C++ binary and open a subprocess call to run it.

## C++ Build Instructions
Alternatively, the raytracer executable can be compiled using the standard CMAKE build scheme. Clone this repository, and then

```bash
cd lens_raytracer_3d
mkdir build
cd build
cmake ..
make
```

## C++ Run Instructions
The current primary executable will simulate rays emitted from an area light source passing through the lens stack described in the `lens_raytracer_3d/config/scene.yaml` file. To run, type:

```bash
./bin/raytracer
```
