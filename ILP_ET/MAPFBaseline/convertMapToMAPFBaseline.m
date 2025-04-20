function convertMapToMAPFBaseline(inputFile)
    % Load the .mat file containing the map object
    load(inputFile, 'map');
    
    % Create output filename by replacing .mat extension with .map
    [~, filename, ~] = fileparts(inputFile);
    outputFile = [filename '.map'];
    
    % Open output file for writing
    fid = fopen(outputFile, 'w');
    if fid == -1
        error('Cannot create output file');
    end
    
    % Write header information
    fprintf(fid, 'type octile\n');
    fprintf(fid, 'height %d\n', map.Height);
    fprintf(fid, 'width %d\n', map.Width);
    fprintf(fid, 'map\n');
    
    % Write map data
    for i = 1:map.Height
        for j = 1:map.Width
            if map.MapGrid(i,j) == 0
                % Free space
                fprintf(fid, '.');
            else
                % Obstacle
                fprintf(fid, '@');
            end
        end
        % New line after each row
        fprintf(fid, '\n');
    end
    
    % Close the file
    fclose(fid);
    
    fprintf('Converted %s to MAPF baseline format successfully!\n', inputFile);
end