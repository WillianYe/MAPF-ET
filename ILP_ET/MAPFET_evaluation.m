clear;
clc;

printInfo = true;

mapFile='map32x32x20%';
instanceFile = ['instance-', mapFile];
load(mapFile)
load(instanceFile);

typeNum = 6;
instanceNum = size(InstanceSet,2);

objectiveSelect = 8;
savePathTotalET=['result-',mapFile,'-totalET'];
try
    load(savePathTotalET)
catch
    Results = cell(typeNum,instanceNum);
end
for typeID = 1:typeNum
    has_solution = false;
    for instanceID = 1:instanceNum
        instance = InstanceSet{typeID,instanceID};
        fprintf("Solving total ET %s-%d-%d with robot number %d\n",instanceFile,typeID,instanceID,instance.RobotNum);
        Results{typeID,instanceID} = MRPP(instance.RobotNum,map,instance.StartRCT,instance.GoalRCT,objectiveSelect,printInfo,0);                   
        save(savePathTotalET,'Results');
        if ~isempty(Results{typeID,instanceID}{1,1}.ObjectiveValue)
            has_solution = true;
        end
    end
    if ~has_solution
        break;
    end
end

objectiveSelect = 9;
savePathMaxET=['result-',mapFile,'-maxET'];
try
    load(savePathMaxET)
catch
    Results = cell(typeNum,instanceNum);
end

for typeID = 1:typeNum
    has_solution = false;
    for instanceID = 1:instanceNum
        if ~isempty(Results{typeID,instanceID})
            continue;
        end
        instance = InstanceSet{typeID,instanceID};
        fprintf("Solving maximum ET %s-%d-%d with robot number %d\n",instanceFile,typeID,instanceID,instance.RobotNum);
        Results{typeID,instanceID} = MRPP(instance.RobotNum,map,instance.StartRCT,instance.GoalRCT,objectiveSelect,printInfo,0);                   
        save(savePathMaxET,'Results'); 
        if ~isempty(Results{typeID,instanceID}{1,1}.ObjectiveValue)
            has_solution = true;
        end
    end
    if ~has_solution
        break;
    end
end


%% 先求解makespan再求解total/maxET
%目的是获取更准确的Tmin，如果Tmin = max(goalRCT(:,3));可能求不出解
%但这样太慢了。还是按MRPP-DT论文里的那样，直接设置Tmin = max(goalRCT(:,3))
%修改MRPP.m20-24行

% objectiveSelect = 0;
% Results = cell(typeNum,instanceNum);
% savePathMakeSpan=['result-',mapFile,'-makespan'];
% load(savePathMakeSpan);
% 
% for typeID = 1:typeNum
%     has_solution = false;
%     for instanceID = 1:instanceNum
%         instance = InstanceSet{typeID,instanceID};
%         fprintf("Solving makespan %s-%d-%d with robot number %d\n",instanceFile,typeID,instanceID,instance.RobotNum);
%         tic;
%         result.Solutions = MRPP(instance.RobotNum,map,instance.StartRCT,instance.GoalRCT,objectiveSelect,printInfo,0);                   
%         result.ComputeTime = toc;  
%         Results{typeID,instanceID} = result;
%         fprintf("Compute Time is %f\n",result.ComputeTime);
%         save(savePathMakeSpan,'Results'); 
%         if ~isempty(result.Solutions{1,1}.ObjectiveValue)
%             has_solution = true;
%         end
%     end
%     if ~has_solution
%         break;
%     end
% end

% objectiveSelect = 8;
% load(savePathMakeSpan);
% ResultsMS=Results;
% Results = cell(typeNum,instanceNum);
% savePathTotalET=['result-',mapFile,'-total-ET'];
% for typeID = 1:typeNum
%     has_solution = false;
%     for instanceID = 1:instanceNum
%         instance = InstanceSet{typeID,instanceID};
%         fprintf("Solving total ET %s-%d-%d with robot number %d\n",instanceFile,typeID,instanceID,instance.RobotNum);
%         msResult = ResultsMS{typeID,instanceID}.Solutions;
%         result = msResult{1,1};
%         Results{typeID,instanceID} = MRPP(instance.RobotNum,map,instance.StartRCT,instance.GoalRCT,objectiveSelect,printInfo,result.T);                   
%         save(savePathTotalET,'Results');
%         if ~isempty(Results{typeID,instanceID}{1,1}.ObjectiveValue)
%             has_solution = true;
%         end
%     end
%     if ~has_solution
%         break;
%     end
% end
% 
% objectiveSelect = 9;
% load(savePathMakeSpan);
% ResultsMS=Results;
% Results = cell(typeNum,instanceNum);
% savePathMaxET=['result-',mapFile,'-maximum-ET'];
% for typeID = 1:typeNum
%     has_solution = false;
%     for instanceID = 1:instanceNum
%         instance = InstanceSet{typeID,instanceID};
%         fprintf("Solving maximum ET %s-%d-%d with robot number %d\n",instanceFile,typeID,instanceID,instance.RobotNum);
%         msResult = ResultsMS{typeID,instanceID}.Solutions;
%         result = msResult{1,1};
%         Results{typeID,instanceID} = MRPP(instance.RobotNum,map,instance.StartRCT,instance.GoalRCT,objectiveSelect,printInfo,result.T);                   
%         save(savePathMaxET,'Results'); 
%         if ~isempty(Results{typeID,instanceID}{1,1}.ObjectiveValue)
%             has_solution = true;
%         end
%     end
%     if ~has_solution
%         break;
%     end
% end

%% test
% load('map2x3.mat')
% load('instances2x3.mat');
% typeNum = 1;
% instanceNum = 1;
% 
% Results = cell(typeNum,instanceNum);
% for typeID = 1:typeNum
%     for instanceID = 1:instanceNum
%         instance = InstanceSet{typeID,instanceID};
%         fprintf("Solving makespan instance-2x3-%d-%d with robot number %d\n",typeID,instanceID,instance.RobotNum);
%         tic;
%         result.Solutions = MRPP(instance.RobotNum,map,instance.StartRCT,instance.GoalRCT,objectiveSelect,printInfo,0);                   
%         result.ComputeTime = toc;  
%         Results{typeID,instanceID} = result;
%         fprintf("Compute Time is %f\n",result.ComputeTime);
%         save('result-2x3-makespan','Results');       
%     end
% end
% 
% objectiveSelect = 9;
% load('result-2x3-makespan.mat');
% ResultsMS=Results;
% Results = cell(typeNum,instanceNum);
% for typeID = 1:typeNum
%     for instanceID = 1:instanceNum
%         instance = InstanceSet{typeID,instanceID};
%         fprintf("Solving total ET instance-2x3-%d-%d with robot number %d\n",typeID,instanceID,instance.RobotNum);
%         msResult = ResultsMS{typeID,instanceID}.Solutions;
%         result = msResult{1,1};
%         Results{typeID,instanceID} = MRPP(instance.RobotNum,map,instance.StartRCT,instance.GoalRCT,objectiveSelect,printInfo,result.T);                   
%         save('result-2x3-total-ET','Results');       
%     end
% end
