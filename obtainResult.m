function [ ] = obtainResult( clouds , motion,new)
%OBTAINRESULT �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
if new
figure;
end
% for i=1:length(clouds)
for i=1:length(motion)
    pcshow(pctransform(clouds{i},affine3d(motion{i}')));
    hold on;
end

