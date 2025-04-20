function convertMAPFBaselineToMap(inputFile)
    % Read the map file
    fid = fopen(inputFile, 'r');
    if fid == -1
        error('Cannot open input file');
    end
    
    % Read header information
    fgetl(fid);
    heightLine = fgetl(fid);
    widthLine = fgetl(fid);
    fgetl(fid);
    
    % Parse dimensions
    height = str2double(regexp(heightLine, '\d+', 'match'));
    width = str2double(regexp(widthLine, '\d+', 'match'));
    
    % Initialize MyGraph object
    map = MyGraph();
    map.Height = height;
    map.Width = width;
    
    % Read map data
    map.MapGrid = zeros(height, width);
    vertexCount = 0;
    freeSpaces = [];
    
    % Process map data
    for i = 1:height
        line = fgetl(fid);
        for j = 1:width
            if line(j) == '.'
                map.MapGrid(i,j) = 0;
                vertexCount = vertexCount + 1;
                freeSpaces = [freeSpaces; i j];
            else
                map.MapGrid(i,j) = 1;
            end
        end
    end
    fclose(fid);
    
    % Calculate obstacle proportion
    totalCells = height * width;
    obstacles = sum(sum(map.MapGrid));
    map.ObstacleProportion = obstacles / totalCells;
    
    % Set vertices
    map.Vertices = freeSpaces;
    map.VertexNum = vertexCount;
    
    % Create VertexIDMat
    map.VertexIDMat = zeros(height, width);
    vertexID = 1;
    for i = 1:height
        for j = 1:width
            if map.MapGrid(i,j) == 0
                map.VertexIDMat(i,j) = vertexID;
                vertexID = vertexID + 1;
            end
        end
    end
    
    % Calculate DistMat using optimized BFS method
    map.DistMat = zeros(map.VertexNum);
    for i = 1:map.VertexNum
        if mod(i,100)==0
            disp(["finish ",i]);
        end
        % 只计算上三角部分
        start = map.Vertices(i,:);
        distMap = calculateDistanceFromNode(map.MapGrid, start, map.VertexIDMat, map.VertexNum);
        % 利用对称性填充距离矩阵
        map.DistMat(i,i:end) = distMap(i:end);
        map.DistMat(i:end,i) = distMap(i:end)';
    end
    
    % Save the object to a .mat file
    [~, filename, ~] = fileparts(inputFile);
    outputFile = [filename '.mat'];
    save(outputFile, 'map');
    
    fprintf('Processed %s successfully!\n', inputFile);
end