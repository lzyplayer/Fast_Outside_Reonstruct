function [ ] = obtainResult( clouds , motion,new)
%OBTAINRESULT 此处显示有关此函数的摘要
%   此处显示详细说明
if new
figure;
end
% for i=1:length(clouds)
for i=1:length(motion)
    pcshow(pctransform(clouds{i},affine3d(motion{i}')));
    hold on;
end

