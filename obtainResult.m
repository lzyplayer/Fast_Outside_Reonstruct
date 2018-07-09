function [ ] = obtainResult( clouds , motion,new)
%OBTAINRESULT 此处显示有关此函数的摘要
%   此处显示详细说明
if new
    figure;
end
if (~isempty(motion{3}) &&  ~isRigid(affine3d(motion{3}')))
    for i=1:length(motion)
        Location=clouds{i}.Location';
        TLocation=motion{i}*[Location;ones(1,size(Location,2))];
        currPointCloud=pointCloud(TLocation(1:3,:)');
        pcshow(currPointCloud);
        hold on;
    end
else
    for i=1:length(motion)
        pcshow(pctransform(clouds{i},affine3d(motion{i}')));
        hold on;
    end
end
% figure;
% for i=238:239
%     pcshow(pctransform(clouds{i},affine3d(motion{i}')));
%     hold on;
% end

