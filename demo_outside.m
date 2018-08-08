% 本算法采用特征点匹配，ICP，回环检测，运动平均算法
clc;clear;close all;

addpath('./flann/');
addpath('./estimateRigidTransform');
icpGridStep = 0.3;
eigDGridStep = 1.4;
% eigLoGridStep=0.03;
filepath='./data/local_frame/';
filePrefix='Hokuyo_';
readnum=30;             %31 in total , start from 0
% scannum=length(dir(datapath))-2;
overlap = 0.35;
icpToler= 0.018;
ICPthreashold= 50;
maxPairDistance=1.2;
res= 10;
s= 1;
% clouds=readCloudCsv(filepath,filePrefix,readnum,0.6 ,s);
load Outside_ori.mat

generalTime=tic;
N = length(clouds);
MotionGlobal{1}=eye(4);
dowmSampleClouds{1}=pcdownsample(pcdenoise(clouds{1}),'gridAverage',icpGridStep);
globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);
LoopDectNum=floor(N/4);
cameraPosePair=[];
LoopFlag=0;
currLoop=0;
ns={};
fixedPointCloudN={};
%% 　主循环
for i=2:N
%   [relativeMotion,MSE]=forwardICP(ns,i,dowmSampleClouds,clouds,ICPthreashold,overlap);
    disp(['processing frame ' num2str(i) '...'])
    dowmSampleClouds{i}=pcdownsample(pcdenoise(clouds{i}),'gridAverage',icpGridStep);
    ns{i-1}=createns(dowmSampleClouds{i-1}.Location,'nsmethod','kdtree');
    IcpModel=[dowmSampleClouds{i-1}.Location';ones(1,dowmSampleClouds{i-1}.Count)];
    IcpData=[dowmSampleClouds{i}.Location';ones(1,dowmSampleClouds{i}.Count)];
    [relativeMotion{i},MSE(i,1)]=myTrimICP(ns{i-1},IcpModel,IcpData,relativeMotion{i-1},ICPthreashold,overlap);
  
    if(MSE(i,1)>icpToler || (i<10 && i>3) )  %单帧配准误差过大,   
          [relativeMotion{i}, MSE(i,1)]=matchFix(clouds{i-1},clouds{i},overlap,eigDGridStep,res,i);
%           fixedPointCloudN=[fixedPointCloudN ,i];
    end
    MotionGlobal{i}=MotionGlobal{i-1}*relativeMotion{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
    %% 回环检测开始
    LoopPairNum=size(cameraPosePair,1);
    if(size(globalCameraPosition,1)>LoopDectNum)
        [cameraPosePair,LoopFlag]=estimateLoopFixed(globalCameraPosition,cameraPosePair,LoopDectNum,LoopFlag,maxPairDistance);
    end
    %% 回环结束_特征点匹配_匹配对扩展
    if((LoopPairNum==size(cameraPosePair,1) || i==N) && (LoopFlag==1 ))
        currLoop=currLoop+1;
        routeDisplay(MotionGlobal,'b-*',true,[]);
        loopNumList(currLoop)=i;
        accMotion=fastDesEigMatch(clouds,cameraPosePair,overlap,eigDGridStep,res,icpToler);
        beforeMotion=(2:length(relativeMotion));
        fixMotion=arrayfun(@(x) {relativeMotion{x},x-1,x} , beforeMotion ,'UniformOutput',false ); %补上前面fix结果
        for f=1:length(fixMotion)
            accMotion=[accMotion;fixMotion{f}];
        end
        D=gen_Dij(accMotion,i,loopNumList);
        updatedGlobalMotion=MotionAverage(accMotion,MotionGlobal,D,size(accMotion,1),i,1,loopNumList);
        for k=1:length(updatedGlobalMotion)
            MotionGlobal{k}=updatedGlobalMotion{k};
        end
        LoopFlag=0;
    end
        
end
generalTime=toc(generalTime)
routeDisplay(MotionGlobal,'r-o',false,[]);
load outside_GRT;
% for g=1:length(GrtM)
% zoomedGrtM{g}=GrtM{g};
% zoomedGrtM{g}(1:3,4)=zoomedGrtM{g}(1:3,4)./s;
% 
% end
routeDisplay(GrtM,'g-s',false,[]);

% obtainResultOri(clouds,GrtM,false);
obtainResultOri(clouds,MotionGlobal,false);
% ransac()
