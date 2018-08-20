function [result] = pcZTransMulti(cloud,currMotion)
%PCTRANSMULTI �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
    Euler=rotm2eul(currMotion(1:3,1:3),'XYZ');
    zEuler=[0 , 0, Euler(3)];
    conRot=eul2rotm( zEuler,'XYZ');
    conTran=[currMotion(1:2,4);0];
    currMotion=Rt2M(conRot,conTran);

    if (isRigid(affine3d(currMotion')))
        result=pctransform(cloud,affine3d(currMotion'));
    else
        Location=cloud.Location';
        TLocation=currMotion*[Location;ones(1,size(Location,2))];
        result=pointCloud(TLocation(1:3,:)');
    end
end
