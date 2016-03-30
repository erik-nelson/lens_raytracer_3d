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


function make_rays(origins, directions)
% Opens the scene file under 'lens_raytracer_3d/config/scene.yaml' and
% writes an array of ray elements. This function can be used to dump a large
% amount of data to the scene file without opening and closing it multiple
% times.
% Input parameters:
%   - origins    : An Nx3 array of ray origins.
%   - directions : An Nx3 array of ray directions.

file_path = '../../config/';
scene_file = 'scene.yaml';

% Check for correct input.
if size(origins, 1) ~= size(directions, 1)
  disp('Number of ray origins and ray directions is not consistent.');
  return;
end

if size(origins, 2) ~= 3 || size(directions, 2) ~= 3
  disp('Origin or direction array is improperly sized.');
  return;
end

% Check if the scene file exists. If not, create it.
if exist([file_path, scene_file], 'file') ~= 2
    reset_scene_file;
end

% Open the scene file.
fh = fopen([file_path, scene_file], 'A');

% Append the ray information to the end of the file.
for ray = 1 : size(origins, 1)
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
      origins(ray, 1), origins(ray, 2), origins(ray, 3),...
      directions(ray, 1), directions(ray, 2), directions(ray, 3));
end

fprintf(1, '\nWrote %d rays to %s.\n', size(origins, 1),...
        [file_path, scene_file]);

% Close the scene file.
fclose(fh);
