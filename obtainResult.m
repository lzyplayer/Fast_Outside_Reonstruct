function [ ] = obtainResult( clouds , motion,new)
%OBTAINRESULT 此处显示有关此函数的摘要
%   此处显示详细说明
if new
    figure;
end

for i=1:length(motion)
    currMotion=motion{i};
    Euler=rotm2eul(currMotion(1:3,1:3),'XYZ');
    zEuler=[0 , 0, Euler(3)];
    conRot=eul2rotm( zEuler,'XYZ');
    conTran=[currMotion(1:2,4);0];
    currMotion=Rt2M(conRot,conTran);

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
    % figure;
    % for i=238:239
    %     pcshow(pctransform(clouds{i},affine3d(motion{i}')));
    %     hold on;
    % end
    
