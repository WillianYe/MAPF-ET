%% convert MAPF baseline
clear;
clc;
% 获取map文件夹中所有的.map文件
mapFiles = dir('maps/*.map');
% 遍历处理每个文件
for i = 1:length(mapFiles)
    % 获取完整的输入文件路径
    inputPath = fullfile('maps/', mapFiles(i).name);
    % 显示正在处理的文件名
    fprintf('Processing %s...\n', mapFiles(i).name);    
    % 调用转换函数
    convertMAPFBaselineToMap(inputPath);
end

fprintf('All maps have been processed!\n');