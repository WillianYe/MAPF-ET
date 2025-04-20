clear;
clc;

% typeNum = 1; %robot number spans 10-90
% instanceNum = 1; %20 instances in each instance type
% InstanceSet = cell(typeNum,instanceNum);
% 
% load('map2x3.mat');
% locations = map.Vertices;
% locationNum = map.VertexNum;
% 
% for typeID=1:typeNum
%     robotNum = typeID*2;
%     colorMat=rand(robotNum,3);
%     for instanceID=1:instanceNum        
%         instance.Map = map;
%         instance.RobotNum = robotNum;
%         instance.ColorMat = colorMat;
% 
%         stations = zeros(2*robotNum,2);
%         vec = randperm(locationNum);
%         for i=1:robotNum*2
%             stations(i,1:2)= locations(vec(i),:);
%         end
%         StartRCT = ones(robotNum,3);
%         GoalRCT = ones(robotNum,3);
%         StartRCT(:,1:2) = stations(1:robotNum,:);
%         GoalRCT(:,1:2) = stations(robotNum+1:end,:);
% 
%         for i=1:robotNum    
%             startID = vec(1,i);
%             goalID = vec(1,robotNum+i);
%             idealTime = map.DistMat(startID,goalID);
%             %dueTime = map.DistMat(startID,goalID)+unidrnd(5);
%             GoalRCT(i,3)=idealTime+1;
%         end
%         instance.StartRCT = StartRCT;
%         instance.GoalRCT = GoalRCT;
%         InstanceSet{typeID,instanceID}=instance;    
%     end
% end
% 
% save('instances2x3.mat','InstanceSet');

%% map parameters and create map

typeNum = 9; %robot number spans 10-90
instanceNum = 20; %20 instances in each instance type
InstanceSet = cell(typeNum,instanceNum);

originalFileNames = {'map20x20x10%', 'map20x20x20%'};
targetFilePrefix = 'instance-';
for i = 1:length(originalFileNames)
    % 构建完整的文件名
    originalFileName = originalFileNames{i};
    targetFileName = [targetFilePrefix, originalFileNames{i}, '.mat'];
    load(originalFileName);
    locations = map.Vertices;
    locationNum = map.VertexNum;

    for typeID=1:typeNum
        robotNum = typeID*10;
        colorMat=rand(robotNum,3);
        for instanceID=1:instanceNum        
            instance.RobotNum = robotNum;
            instance.ColorMat = colorMat;

            stations = zeros(2*robotNum,2);
            vec = randperm(locationNum);
            for i=1:robotNum*2
                stations(i,1:2)= locations(vec(i),:);
            end
            StartRCT = ones(robotNum,3);
            GoalRCT = ones(robotNum,3);
            StartRCT(:,1:2) = stations(1:robotNum,:);
            GoalRCT(:,1:2) = stations(robotNum+1:end,:);

            for i=1:robotNum    
                startID = vec(1,i);
                goalID = vec(1,robotNum+i);
                idealTime = map.DistMat(startID,goalID);
                %dueTime = map.DistMat(startID,goalID)+unidrnd(5);
                GoalRCT(i,3)=idealTime+1;
            end
            instance.StartRCT = StartRCT;
            instance.GoalRCT = GoalRCT;
            InstanceSet{typeID,instanceID}=instance;    
        end
    end
    save(targetFileName,'InstanceSet');    
end

