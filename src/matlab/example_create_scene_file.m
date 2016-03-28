% This script provides an example for how to create a scene file for
% raytracing. An image of the resulting raytraced scene is shown in
% 'simulation.png' in the top directory of this repository.
clc; close all; clear;

% Clear whatever scene file currently exists.
reset_scene_file;

% Add lenses to the scene, simulating a camera lens stack.
n = 1.53; % Index of refraction for all lenses.
make_lens(1.5, inf, 0.1, 1.0, n, [0, 0, 0.38], [0, 0, 0]);
make_lens(0.75, -2.0, 0.1, 0.8, n, [0, 0, 0.28], [0, 0, 0]);
make_lens(3.0, -0.6, 0.01, 0.8, n, [0, 0, 0.2], [0, 0, 0]);
make_lens(-0.6, inf, 0.01, 0.8, n, [0, 0, -0.2], [0, 0, 0]);
make_lens(inf, 1.0, 0.15, 0.8, n, [0, 0, -0.285], [0, 0, 0]);
make_lens(-3.0, 0.9, 0.1, 0.9, n, [0, 0, -0.42], [0, 0, 0]);
make_lens(3.0, 5.0, 0.07, 1.0, n, [0, 0, -0.52], [0, 0, 0]);

% Make some rays in a circle pattern facing towards the lens stack
% (negative z direction).
for a = 0 : (pi/4) : 2*pi
    for radius = 0.2 : 0.2 : 0.4
        x = radius * cos(a);
        y = radius * sin(a);
        make_ray([x, y, 2.0], [0, 0, -1.0]);
    end
end

% Run the raytracer via a subprocess call to the C++ executable.
run_raytracer;
