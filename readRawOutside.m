function [clouds ] = readRawOutside( filepath,filePrefix,scannum ,s )
%READROOM �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
    count=scannum-1; %��0��ʼ
%% ��ȡ����
    clouds=readCloudCsv(filepath,filePrefix,count,0.02,s); %ȥ�������
%     count=count+1��
% transmatrix=[-1 0 0 0
%              0 0 -1 0
%              0 -1 0 0
%              0 0 0 1];   %%���Դ���ת
%     clouds{2}=pctransform(clouds{2},affine3d(transmatrix'));
    
%     for j=1:scannum
% %%  ��ȡ����
% %         clouds{j}=pointCloud(data{j}(:,1:3)./1000);
% %         [srcDesp{j},srcSeed{j},srcNorm{j}] = extractEig(clouds{j},gridstep);
%     end
end

