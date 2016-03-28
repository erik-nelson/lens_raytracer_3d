% This script opens a subprocess running the C++ raytracer executable. If
% the executable does not yet exist, it runs a subprocess to compile the
% executable and then runs it.

% Check if the raytracer binary exists.
executable = '../../bin/raytracer';
if exist(executable', 'file')~=2
    % The executable doesn't exist, so we should compile it.
    cmd = ['mkdir ../../build; '...
           'cd ../../build; '...
           'cmake -DCMAKE_BUILD_TYPE=Release ..; '...
           'make; '...
           'cd ../src/matlab; '];
    system(cmd);
end

% Run the executable.
system(executable);