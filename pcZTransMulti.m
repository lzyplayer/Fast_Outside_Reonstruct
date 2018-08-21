function [resultPC] = pcZTransMulti(cloud,currMotion)
%PCTRANSMULTI 此处显示有关此函数的摘要
%   此处显示详细说明
    Euler=rotm2eul(currMotion(1:3,1:3),'XYZ');
    zEuler=[0 , 0, Euler(3)];
    conRot=eul2rotm( zEuler,'XYZ');
    conTran=[currMotion(1:2,4);0];
    currMotion=Rt2M(conRot,conTran);

    if (isRigid(affine3d(currMotion')))
        resultPC=pctransform(cloud,affine3d(currMotion'));
    else
        Location=cloud.Location';
        TLocation=currMotion*[Location;ones(1,size(Location,2))];
        resultPC=pointCloud(TLocation(1:3,:)');
    end
end

