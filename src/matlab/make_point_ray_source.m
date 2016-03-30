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


function make_point_ray_source(point, direction, v_angles, h_angles)
% Opens the scene file under 'lens_raytracer_3d/config/scene.yaml' and
% writes an array of ray elements originating from the input point. Rays
% will be generated beginning from the specified direction in a cone
% outwards according to the specified vertical and horizontal angles.
% Input parameters:
%   - point     : A point in 3D space from which rays will originate.
%   - direction : A 3-element direction vector specifying the center 
%                 direction from which rays will eminate.
%   - v_angles  : A list of vertical angles to generate rays (outwards from
%                 the specified direction).
%   - h_angles  : A list of horizontal angles to generate rays (outwards
%                 from the specified direction).

% Check for correct input.
if numel(point) ~= 3
    disp('Point argument must be a 3D point.');
    return;
end

if numel(direction) ~= 3
    disp('Direction argument must be a 3D vector.');
    return;
end

% Find one vector orthogonal to the direction vector w/ Gram Schmidt.
proj = @(x, y) x'*y / (y'*y) * y;
r = rand(3,1);
u = r(:,1) - proj(r(:,1), direction);
u = u./norm(u);

% Define rotation matrices about the direction and its orthogonal vector.
rot_d = @(theta) vrrotvec2mat([direction; theta]);
rot_u = @(theta) vrrotvec2mat([u; theta]);

% Initialize containers for writing out rays.
n_rays = numel(v_angles) * numel(h_angles);
origins = repmat(point, 1, n_rays)';
directions = zeros(n_rays, 3);

% Compute ray directions based on input.
for ii=1:numel(v_angles)
    d = rot_u(v_angles(ii)) * direction;
    for jj=1:numel(h_angles)
        index = (ii-1) * numel(h_angles) + jj;
        directions(index, :) = (rot_d(h_angles(jj)) * d)'; 
    end
end

% Normalize output directions.
directions = normr(directions);

% Write ray parameters to the scene file.
make_rays(origins, directions);