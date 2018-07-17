function [turnFlag,zAxis,planeAxis] = turnDectecion(currMotion,turnThreshold)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
turnFlag=false;
turn=rotm2eul(currMotion(1:3,1:3),'XYZ');
zAxis=abs(turn(3));
if zAxis>=turnThreshold
    turnFlag=true;
end
planeAxis=max(abs(turn(1:2)));
end

