% 本算法
clc;clear;close all;

addpath('./flann/');
addpath('./estimateRigidTransform');
icpGridStep = 0.01;
eigDGridStep = 0.04;
% eigLoGridStep=0.03;
filepath='./data/local_frame/';
filePrefix='Hokuyo_';
readnum=31;
% scannum=length(dir(datapath))-2;
overlap = 0.35;
icpToler=2e-05;
ICPthreashold=50;
res= 10;
s=30;
load trimOutside.mat
tic;
N = length(clouds);
MotionGlobal{1}=eye(4);
dowmSampleClouds{1}=pcdownsample(pcdenoise(clouds{1}),'gridAverage',icpGridStep);
globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);
LoopDectNum=floor(N/4);
cameraPosePair=[];
LoopFlag=0;
ns={};
%% 　主循环
for i=2:N
%   [relativeMotion,MSE]=forwardICP(ns,i,dowmSampleClouds,clouds,ICPthreashold,overlap);
    dowmSampleClouds{i}=pcdownsample(pcdenoise(clouds{i}),'gridAverage',icpGridStep);
    [currMotion2next]=pcregrigid(dowmSampleClouds{i-1},dowmSampleClouds{i},'Tolerance',[0.01/s,0.009]);
    ns{i-1}=createns(dowmSampleClouds{i-1}.Location,'nsmethod','kdtree');
    IcpModel=[dowmSampleClouds{i-1}.Location';ones(1,dowmSampleClouds{i-1}.Count)];
    IcpData=[dowmSampleClouds{i}.Location';ones(1,dowmSampleClouds{i}.Count)];
    [relativeMotion{i},MSE(i,1)]=myTrimICP(ns{i-1},IcpModel,IcpData,relativeMotion{i-1},ICPthreashold,overlap);
    fixTimes=0; enhanceModelCloud=dowmSampleClouds{i-1};
    if(MSE(i,1)>icpToler )   %单帧配准误差过大,
          [relativeMotion{i}, MSE(i,1)]=matchFix(clouds{i-1},clouds{i},overlap,eigDGridStep,res);
    end
    MotionGlobal{i}=MotionGlobal{i-1}*relativeMotion{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
    %% 回环检测开始
    LoopPairNum=size(cameraPosePair,1);
    if(size(globalCameraPosition,1)>LoopDectNum)
        [cameraPosePair,LoopFlag]=estimateLoop(globalCameraPosition,cameraPosePair,LoopDectNum,LoopFlag);
    end
    %% 回环结束_特征点匹配_匹配对扩展
    if((LoopPairNum==size(cameraPosePair,1) || i==N) && (LoopFlag==1 ))
        accMotion=fastDesEigMatch(clouds,cameraPosePair,overlap,eigDGridStep,res);
        D=gen_Dij(accMotion,i);
        updatedGlobalMotion=MotionAverage(accMotion,MotionGlobal,D,size(accMotion,1),i);
        for k=1:length(updatedGlobalMotion)
            MotionGlobal{k}=updatedGlobalMotion{k};
        end
        LoopFlag=0;
    end
        
end
toc

% [clouds] = readRawOutside(filepath,filePrefix,readnum,s);
obtainResult(clouds,MotionGlobal);

