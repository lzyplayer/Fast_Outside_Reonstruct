% 本算法采用特征点匹配，ICP，回环检测，运动平均算法
clc;clear;close all;

addpath('./flann/');
addpath('./estimateRigidTransform');
icpGridStep = 0.3;
eigDGridStep = 0.3287; %0.3287
mergeGridStep=0.01;      %%test for now
% filepath='./data/IAIR303/';
% filePrefix='resultIAIR303_1.txt';

% scannum=length(dir(datapath))-2;
overlap = 0.5;
icpToler= 1e-4;
ICPthreashold= 50;
maxPairDistance=0.1;
res= 10;

s= 1;
% clouds=readCloudCsv(filepath,filePrefix,readnum,0.6 ,s);
load IAIR303_ourside_trimmed.mat

generalTime=tic;
N = length(clouds);
MotionGlobal{1}=eye(4);
denoisedClouds{1}=pcdenoise(clouds{1});
globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);
LoopDectNum=15;
cameraPosePair=[];
LoopFlag=0;
ns={};
fullCloud=denoisedClouds{1};

%% axes获取
curFig=figure('Position',[7 76 1382 919]);
reAxes=pcshow(fullCloud);
axis(reAxes,[-6.5 2 -2.3 4 -0.75 1.3])
title('Slam On IAIR303')
% reAxes.CameraViewAngleMode = 'auto';
reAxes.CameraPosition=[-2.868 -1.946 53.981];
reAxes.CameraTarget=[-2.279 0.809 0.219];
reAxes.CameraUpVector=[-0.217 -0.975 -0.053];
reAxes.CameraViewAngle=8.64562;
reScatter = reAxes.Children;


routeFig=figure('Position',[1327.0 76 587.0 919]);
routeAxes=axes();
axis(routeAxes,[-4 1 -1 3 -0.5 0.5]);
title('Slam Route On IAIR303')
% reAxes.CameraViewAngleMode = 'manual';
routeAxes.CameraPosition=[-1.5 1 32.404];
routeAxes.CameraTarget=[-1.5 1 0];
routeAxes.CameraUpVector=[-1.542 -3.805 0];
routeAxes.CameraViewAngle=10.771;

%% 　主循环
for i=2:N
    %   [relativeMotion,MSE]=forwardICP(ns,i,dowmSampleClouds,clouds,ICPthreashold,overlap);
    %     denoisedClouds{i}=clouds{i};
    %     Model=denoisedClouds{i-1}.Location';
    %     Data=denoisedClouds{i}.Location';
    %     R0=relativeMotion{i-1}(1:3,1:3);
    %     t0=relativeMotion{i-1}(1:3,4);
    %     [MSE(i,1),R,t] = TrICP(Model, Data, R0, t0, ICPthreashold, overlap);
    %     relativeMotion{i}=Rt2M(R,t);
    %     if(MSE(i,1)>icpToler )  %单帧配准误差过大,
    %        downSampleCloud{i}=pcdownsample(clouds{1},'gridAverage',)
    [relativeMotion{i}, MSE(i,1)]=matchFix(clouds{i-1},clouds{i},overlap,eigDGridStep,res);
    disp(['cloud ' num2str(i-1) '-' num2str(i) ' matched!']);
    %           fixedPointCloudN=[fixedPointCloudN ,i];
    %     end
    MotionGlobal{i}=MotionGlobal{i-1}*relativeMotion{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
    
        %% anime
    
    fullCloud = pcmerge(fullCloud,pcZTransMulti(clouds{i},MotionGlobal{i}),mergeGridStep);
    reScatter.XData=fullCloud.Location(:,1);
    reScatter.YData=fullCloud.Location(:,2);
    reScatter.ZData=fullCloud.Location(:,3);
    reScatter.CData=fullCloud.Location(:,3);
    drawnow()
    
    routeHandle=routeAnimePlugin(  MotionGlobal,'r-o',routeAxes);
    
    %% 回环检测开始
    LoopPairNum=size(cameraPosePair,1);
    if(size(globalCameraPosition,1)>LoopDectNum)
        [cameraPosePair,LoopFlag]=estimateLoop(globalCameraPosition,cameraPosePair,LoopDectNum,maxPairDistance,LoopFlag);
    end
    %% 回环结束_特征点匹配_匹配对扩展
    if((LoopPairNum==size(cameraPosePair,1) || i==N) && (LoopFlag==1 ))
        routeDisplay(MotionGlobal,'b-*',true,[]);
        accMotion=fastDesEigMatch(clouds,cameraPosePair,overlap,eigDGridStep,res);
        beforeMotion=(2:length(relativeMotion));
        fixMotion=arrayfun(@(x) {relativeMotion{x},x-1,x} , beforeMotion ,'UniformOutput',false ); %补上前面fix结果
        for f=1:length(fixMotion)
            accMotion=[accMotion;fixMotion{f}];
        end
        D=gen_Dij(accMotion,i);
        updatedGlobalMotion=MotionAverage(accMotion,MotionGlobal,D,size(accMotion,1),i);
        for k=1:length(updatedGlobalMotion)
            MotionGlobal{k}=updatedGlobalMotion{k};
        end
        LoopFlag=0;
    end
    
end
generalTime=toc(generalTime)
% routeDisplay(MotionGlobal,'r-o',false,[]);


% load IAIR303.mat
% obtainResult(clouds,MotionGlobal,false);

