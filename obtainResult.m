function [ ] = obtainResult( clouds , motion)
%OBTAINRESULT �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

figure;
for i=1:length(clouds)
    pcshow(pctransform(clouds{i},affine3d(motion{i}')));
    hold on;
end

