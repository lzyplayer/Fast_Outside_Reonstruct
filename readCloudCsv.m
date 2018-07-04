function [ pointClouds ] = readCloudCsv(filepath,filePrefix,readnum ,zlimit,s )
%REACLOUDCSV 此处显示有关此函数的摘要
%   此处显示详细说明 
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
    
    pointGroud= cloud.data(:,z)<zlimit ;% 会拍到机器人自身的部分形成大干扰
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

