function   obtainResultOri(clouds,motion,new)
%OBTAINRESULTORI 此处显示有关此函数的摘要
%   此处显示详细说明
if new
    figure;
end
for i=1:length(clouds)
    currMotion=motion{i};
    if (isRigid(affine3d(currMotion')))
        pcshow(pctransform(clouds{i},affine3d(currMotion')));
        hold on;
    else
        Location=clouds{i}.Location';
        TLocation=currMotion*[Location;ones(1,size(Location,2))];
        currPointCloud=pointCloud(TLocation(1:3,:)');
        pcshow(currPointCloud);
        hold on;
    end
end

