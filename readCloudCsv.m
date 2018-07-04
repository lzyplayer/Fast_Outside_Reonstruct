function [ pointClouds ] = readCloudCsv(filepath,filePrefix,readnum ,zlimit,s )
%REACLOUDCSV �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵�� 
if(readnum>30) 
    error('OutOfRange!');
end
for i=0:readnum
%     filename=[filepath 'PointCloud' num2str(i) '.csv'];
    filename=[filepath filePrefix num2str(i) '.csv'];
    % Load data
    cloud = importdata(filename);

    % Extract coordinates
    x = strcmp('x', cloud.colheaders);
    y = strcmp('y', cloud.colheaders);
    z = strcmp('z', cloud.colheaders);
    
    pointGroud= cloud.data(:,z)<zlimit ;% ���ĵ�����������Ĳ����γɴ����
    pointrobat= cloud.data(:,z)>zlimit & cloud.data(:,z)<0.023*30 & cloud.data(:,x)<0.06*30 & cloud.data(:,y)<0.025*30 & cloud.data(:,y)>-0.025*30;
    pointBad=pointGroud | pointrobat;
    pointSelect=~pointBad;
    
    %     pointSelect= cloud.data(:,4)>-11 ;
%     pointSelect=xor(pointSelectLoud,pointrobat);
%     pointLocation=cloud.data(:,2:4)./s;
    pointLocation=cloud.data(pointSelect,2:4)./s;
    pointClouds{(i+1)}= pointCloud(pointLocation);
end
end

