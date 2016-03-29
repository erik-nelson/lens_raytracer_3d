% Copyright (c) 2016, The Regents of the University of California (Regents).
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:
%
%    1. Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%
%    2. Redistributions in binary form must reproduce the above
%       copyright notice, this list of conditions and the following
%       disclaimer in the documentation and/or other materials provided
%       with the distribution.
%
%    3. Neither the name of the copyright holder nor the names of its
%       contributors may be used to endorse or promote products derived
%       from this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
%
% Please contact the author(s) of this library if you have any questions.
% Author: Erik Nelson            ( eanelson@eecs.berkeley.edu )


% This script provides an example for how to create a scene file for
% raytracing.
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

% Make an array of micro lenses.
for x = -0.45 : 0.1 : 0.45
    for y = -0.45 : 0.1 : 0.45
        make_lens(0.1, inf, 0.03, 0.1, n, [x, y, -3.0], [0, 0, 0]);
    end
end

% Generate two sets of rays passing through different microlenses.
focal_len = 0.188679;
for a = 0 : (pi/4) : 2*pi
    for radius = 0.01 : 0.05 : 0.2
        x = radius * cos(a);
        y = radius * sin(a);
        direction = [radius * cos(a), radius * sin(a), 1];
        direction = direction ./ norm(direction);
        make_ray([-0.15, 0.15, -3.0 - focal_len], direction);
        make_ray([0.25, -0.25, -3.0 - focal_len], direction);
    end
end

% Run the raytracer via a subprocess call to the C++ executable.
run_raytracer;
