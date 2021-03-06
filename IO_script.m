

%% .dat , .pose类型点云读取
filePath='./data/freiburgCampus/';
scanNum=77;
for i=1:scanNum
    if(i>9)
        currNum=['0' int2str(i)];
    else
        currNum=['00' int2str(i)];
    end
    currDat=importdata([filePath 'scan_' currNum '_points.dat']);
    currPDat=importdata([filePath 'scan_' currNum '_robotPoses.dat']);
    clouds{i}=pointCloud(currDat(:,4:6));
    rtMat=eul2tform(currPDat(2,4:6),'XYZ');
    rtMat(1:3,4)=currPDat(2,1:3)';
    gslamResult{i}=rtMat;
%     scan_001_robotPoses.dat
end
obtainResult(clouds,gslamResult,true,0.03);
routeDisplay(gslamResult,'-or',true,[]);

%% velodye 数据读取
filePath='./data/IAIR303/ourSide/resultIAIR303_';
scannum=13;
for i=1:scannum
    indata=importdata([filePath num2str(i) '.txt']);
    pointsFar=sqrt(indata(:,1).^2+indata(:,2).^2)>4.5;
    pointsGround=indata(:,3)<-0.65;
    pointSelect=~(pointsFar | pointsGround);
    clouds{i}=pointCloud(indata(pointSelect,1:3));
end


%% 
clc;clear;
close all;

filepathWp='G:\dc_Rpcr\multi_method_env_reconstruct\Fast_Outside_Reonstruct\data\hannover2\hannover2\scan3d_0_';
% dirInfo=dir('./data/dat_et4/');
scannum=924   ;%(length(dirInfo)-2)/2;
% lastnum=scannum-1;
route=[];
% clouds=cell(scannum,1);
for k=2%:scannum
    ordernum=num2str(k);
%     while(length(ordernum)<3)
%         ordernum=['0' ordernum];
%     end
    prefix=[ordernum '.3d'];
    %读取.3d
    data_raw=importdata([filepathWp prefix]);
    objectbad=(data_raw(:,1).^2+data_raw(:,2).^2+data_raw(:,3).^2>850e+6);
    objectGround=(data_raw(:,3)>(-0.3*1e+3));
    objectSelect=~(objectbad |objectGround);
    
    clouds{k}=pointCloud(data_raw(objectSelect,1:3).*1e-2);
%     
%     clouds{k}=pointCloud(data_raw(:,1:3));
%     pcshow(clouds{k});
%     hold on
    %读取.pose  route only
%         prefix=[ordernum '.pose'];
%         scanpose{k+1}=load([filepathWp prefix]);
% %         R=OulerToRota(scanpose{k+1}(2,:));
%         T=scanpose{k+1}(1,:)';
% %         Grt{k+1}=[R,T;0 0 0 1];
%         route=[route ; T'];
end
save hannover2zoomed.mat clouds;

%% 读取里程计数据
odometry=odometry0syncinterpol(23:end , :);

for i=1:size(odometry,1)
    t=odometry(i,1:3)';
    R=quat2rotm(odometry(i,4:7));
    Motion1(i)=inv(Rt2M(R,t));  %需要考量
    firstCentralMotion(i)=(Motion1(1))\Motion1(i);
end

%% 读取真值 start from frame22
s=1e-1;
filepathWp='./data/hannover2/6Dreg.dat';
indata=load(filepathWp,'X6Dreg');
GrtM=cell(size(indata,1),1);
for i=1:size(indata,1)
    GrtM{i}=[indata(i,1:4);indata(i,5:8);indata(i,9:12);indata(i,13:15).*s,indata(i,16)]';
end
routeDisplay(GrtM,'b-*',true );


%% 逐行变换 不好
%     fidin = fopen([filepathWp prefix],'r');
%     nline = 0;
%     currscan=zeros(81360,4);
%     while ~feof(fidin)         %判断是否为文件末尾
%         nline = nline+1;
%                         
%         tline = fgetl(fidin);         %从文件读行
%         if(nline>1)
%         currscan(nline-1,:) = str2num(tline);
%         end
%     end
%       fclose(fidin);
%     scan{k+1}=currscan;

% pose =importdata('./data/hannover2/hannover2/odometry_0_sync_interpol.dat');

