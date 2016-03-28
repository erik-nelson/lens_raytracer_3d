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


function make_ray(origin, direction)
% Opens the scene file under 'lens_raytracer_3d/config/scene.yaml' and
% writes a new ray element.
% Input parameters:
%   - origin    : A 3-element vector specifying the origin of the ray.
%   - direction : A 3-element vector specifying the direction of the ray.

file_path = '../../config/';
scene_file = 'scene.yaml';

% Check if the scene file exists. If not, create it.
if exist([file_path, scene_file], 'file') ~= 2
    reset_scene_file;
end

% Open the scene file.
fh = fopen([file_path, scene_file], 'at');

% Append the lens information to the end of the file.
fprintf(fh, ['ray:'...
    '\n\torigin:'...
    '\n\t\tx: %.3f'...
    '\n\t\ty: %.3f'...
    '\n\t\tz: %.3f'...
    '\n\tdirection:'...
    '\n\t\tx: %.3f'...
    '\n\t\ty: %.3f'...
    '\n\t\tz: %.3f'...
    '\n'],...
    origin(1), origin(2), origin(3),...
    direction(1), direction(2), direction(3));

fprintf(1, ['\nWrote the following ray to %s:'...
     '\n\torigin:\n\t\tx: %.2f\n\t\ty: %.2f\n\t\tz: %.2f'...
     '\n\tdirection:\n\t\tx: %.2f\n\t\ty: %.2f\n\t\tz: %.2f\n'],...
     [file_path, scene_file],...
     origin(1), origin(2), origin(3),...
     direction(1), direction(2), direction(3));

% Close the scene file.
fclose(fh);
