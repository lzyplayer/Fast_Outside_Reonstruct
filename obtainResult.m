function [ ] = obtainResult( clouds , motion)
%OBTAINRESULT 此处显示有关此函数的摘要
%   此处显示详细说明

figure;
for i=1:length(clouds)
    pcshow(pctransform(clouds{i},affine3d(motion{i}')));
    hold on;
end

