% Open up the 'lens_raytracer_3d/config/scene.yaml' file and clear its
% contents. If the scene file doesn't exist, this script will create an 
% empty file.

file_path = '../../config/';
scene_file = 'scene.yaml';

% Check if the file exists already and delete if it does.
if exist([file_path, scene_file], 'file') == 2
    delete([file_path, scene_file]);
end

% Touch a new version of the scene file.
fh = fopen([file_path, scene_file], 'w');    
fclose(fh);

fprintf(1, 'Created empty file under %s.\n',...
        [file_path, scene_file]);