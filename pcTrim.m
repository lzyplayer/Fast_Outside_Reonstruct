function [trimmedClouds] = pcTrim(clouds,distance,zlimit,downgridstep)
%PCTRIM 此处显示有关此函数的摘要
%   此处显示详细说明
for i=1:length(clouds)
    allInpoints = clouds{i}.Location;
    pointsFar = sqrt(allInpoints(:,1).^2+allInpoints(:,2).^2)>distance;
    pointsGround = (allInpoints(:,3)<zlimit(1)) | (allInpoints(:,3)>zlimit(2));
    pointSelect = ~(pointsFar | pointsGround);
    trimmedClouds{i} = pcdownsample(pcdenoise(pointCloud(allInpoints(pointSelect,:))),'gridAverage',downgridstep);
end
end

