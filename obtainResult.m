function [ ] = obtainResult( clouds , motion)
%OBTAINRESULT �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

figure;
% for i=1:length(clouds)
for i=1:12
    pcshow(pctransform(clouds{i},affine3d(motion{i}')));
    hold on;
end

