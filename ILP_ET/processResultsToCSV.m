function processResultsToCSV(inputFile)
    % Load the .mat file
    data = load(inputFile);
    Results = data.Results;
    
    % Get typeNum from Results size
    [typeNum, instanceNum] = size(Results);
    
    % Extract map name and target name from input file name
    [~, fileName, ~] = fileparts(inputFile);
    parts = split(fileName, '-');
    mapName = parts{2};
    targetName = replace(parts{3}, '.mat', '');
    approach = ['ILP_ET_', targetName, '_0'];
    
    % Check if record.csv exists and create with headers if not
    if ~isfile('record.csv')
        fid = fopen('record.csv', 'w');
        fprintf(fid, 'approach,map name,agent number,success rate,runtime,solution cost,solution obj\n');
        fclose(fid);
    end
    
    % Process each row of Results
    fid = fopen('record.csv', 'a');
    for i = 1:typeNum
        agentNumber = 10 * i;
        successCount = 0;
        validComputeTimes = [];
        validObjectiveValues = [];
        
        % Process each instance in the row
        for j = 1:instanceNum
            if ~isempty(Results{i,j})
                % Find the element with minimum ObjectiveValue and ComputeTime <= 120
                minObjValue = inf;
                minComputeTime = inf;
                hasValidSolution = false;
                
                for k = 1:size(Results{i,j}, 1)
                    result = Results{i,j}{k,1};
                    if ~isempty(result.ObjectiveValue) && ...
                       result.ComputeTime <= 120
                        if result.ObjectiveValue < minObjValue || ...
                          (result.ObjectiveValue == minObjValue && ...
                           result.ComputeTime < minComputeTime)
                            minObjValue = result.ObjectiveValue;
                            minComputeTime = result.ComputeTime;
                            hasValidSolution = true;
                        end
                    end
                end
                
                if hasValidSolution
                    successCount = successCount + 1;
                    validComputeTimes = [validComputeTimes; minComputeTime];
                    validObjectiveValues = [validObjectiveValues; minObjValue];
                end
            end
        end
        
        % Calculate averages and write to CSV
        if successCount > 0
            avgRuntime = mean(validComputeTimes);
            avgObjValue = mean(validObjectiveValues);
            
            fprintf(fid, '%s,%s,%d,%d,%.4f,0,%.4f\n', ...
                approach, ...
                mapName, ...
                agentNumber, ...
                successCount, ...
                avgRuntime, ...
                avgObjValue);
        end
    end
    fclose(fid);
end