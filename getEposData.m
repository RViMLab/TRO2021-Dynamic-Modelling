function [recordedData, samplingPeriod, expEndTime, counts, velocityAvg, torqueAvg, velocity, torque] = ...
    getEposData(dirName, fileName, motorNominalTorque)
    
    [~,~,rawData] = xlsread(fullfile(dirName, fileName));
    rawData = rawData(:,1);
    
    index = find(contains(rawData,'ms'));
    recordedDataLine = index - 1;
    numericDataStart = index + 1;
    
    % samplesNo = str2double(cell2mat(regexp(rawData{7}, '\d*', 'Match')));
    samplingPeriod = str2double(cell2mat(regexp(rawData{8}, '\d*', 'Match')))/10^6;
    recordedData = strsplit(rawData{recordedDataLine}, ';');
    recordedDataNo = numel(recordedData);
    responseData = zeros(length(rawData(numericDataStart:end)), recordedDataNo);
    
    for i = 1:length(rawData(numericDataStart:end))

        responseData(i,:) = (str2num(rawData{numericDataStart - 1 + i})');

    end
    
    countsIdx = find(strcmpi(recordedData, 'position actual value') == 1, 1);
    torqueAveragedIdx = find(strcmpi(recordedData, 'torque actual value averaged') == 1, 1);
    velocityAveragedIdx = find(strcmpi(recordedData, 'velocity actual value averaged') == 1, 1);
    velocityIdx = find(strcmpi(recordedData, 'velocity actual value') == 1, 1);
    torqueIdx = find(strcmpi(recordedData, 'torque actual value') == 1, 1);
    
    time = (0:samplingPeriod:(length(rawData(numericDataStart:end))-1)*samplingPeriod)'; % sec
    expEndTime = max(time); % sec
    counts = timeseries(-responseData(:,countsIdx), time);
    velocityAvg = timeseries(-responseData(:,velocityAveragedIdx)*pi/30, time); % rad/s
    torqueAvg = timeseries(-responseData(:,torqueAveragedIdx)*motorNominalTorque/100, time); % Nm
    velocity = timeseries(-responseData(:,velocityIdx)*pi/30, time); % rad/s
    torque = timeseries(-responseData(:,torqueIdx)*motorNominalTorque/100, time); % Nm
    
end

