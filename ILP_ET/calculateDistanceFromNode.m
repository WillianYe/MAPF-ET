function distMap = calculateDistanceFromNode(mapGrid, start, vertexIDMat, vertexNum)
    [height, width] = size(mapGrid);
    distMap = inf(1, vertexNum);  % 距离初始化为无穷大
    
    % 获取起始节点的ID
    startID = vertexIDMat(start(1), start(2));
    distMap(startID) = 0;
    
    % 定义方向数组：上、右、下、左
    dx = [-1, 0, 1, 0];
    dy = [0, 1, 0, -1];
    
    % 使用队列进行BFS
    queue = {start};
    visited = zeros(height, width);
    visited(start(1), start(2)) = 1;
    
    while ~isempty(queue)
        curr = queue{1};
        queue(1) = [];
        
        % 获取当前节点的ID和距离
        currID = vertexIDMat(curr(1), curr(2));
        currDist = distMap(currID);
        
        % 检查所有相邻节点（只有上下左右四个方向）
        for i = 1:4
            newX = curr(1) + dx(i);
            newY = curr(2) + dy(i);
            
            % 检查是否在地图范围内
            if newX >= 1 && newX <= height && newY >= 1 && newY <= width
                % 检查是否是可行走节点且未访问过
                if mapGrid(newX, newY) == 0 && ~visited(newX, newY)
                    visited(newX, newY) = 1;
                    queue{end+1} = [newX, newY];
                    
                    % 更新距离（每步距离为1）
                    newID = vertexIDMat(newX, newY);
                    distMap(newID) = currDist + 1;
                end
            end
        end
    end
end