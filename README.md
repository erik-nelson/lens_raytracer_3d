# 3D Lens Raytracer

This repository holds a raytracing library for simulating and testing camera lens stacks.

## Status
This library is currently in development.

## Overview
This repository is entirely written in C++, and is structured around the CMAKE cross-compilation paradigm. All source code is in the `lens_raytracer_3d/src/cpp/` directory.

## Build Instructions
We follow the standard CMAKE build scheme. Just download the repository and from the top directory type:

```bash
mkdir build
cd build
cmake ..
 make
```

## Execution instructions
The current primary executable will simulate rays emitted from an area light source passing through the lens stack described in the `lens_raytracer_3d/config/scene1.yaml` file. To run, type:

```bash
./bin/raytracer
```
