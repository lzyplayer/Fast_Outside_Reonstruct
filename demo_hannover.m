% 本算法采用特征点匹配，ICP，回环检测，运动平均算法
clc;clear;close all;

addpath('./flann/');
addpath('./estimateRigidTransform');
eigDGridStep = 30;
% eigLoGridStep=0.03;

readnum=942;
% scannum=length(dir(datapath))-2;
overlap = 0.3;
icpToler= 10;
ICPthreashold= 200;
maxPairDistance=22;%22
MseHold=10;
res= 1;
s= 1;
% clouds=readCloudCsv(filepath,filePrefix,readnum,0.6 ,s);
load hannover2_MZ.mat

generalTime=tic;
N = length(clouds);
MotionGlobal{1}=eye(4);
globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);
LoopDectNum=80;%floor(N/4);
cameraPosePair=[];
LoopFlag=0;
ns={};
lastLoopNum=0;   %新思路！！！ 回环检测阈值应当是变化量
currLoop=0;
fixedPointCloudN={};
%% 　主循环
N=330;
for i=2:N
%     if i==239
%         0==0;
%     end
    Model=clouds{i-1}.Location';
    Data=clouds{i}.Location';
    R0=relativeMotion{i-1}(1:3,1:3);
    t0=relativeMotion{i-1}(1:3,4);
    [MSE(i,1),R,t] = TrICP(Model, Data, R0, t0, ICPthreashold, overlap);
    relativeMotion{i}=Rt2M(R,t);
    if(MSE(i,1)>icpToler || i==239 )   %单帧配准误差过大,     239得修复
        [Motion, mse]=matchFix(clouds{i-1},clouds{i},overlap,eigDGridStep,res);
        if(~isempty(Motion))
            relativeMotion{i}=Motion;MSE(i,1)=mse;
        end
    end
    MotionGlobal{i}=MotionGlobal{i-1}*relativeMotion{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
    
    
    
    %% 回环检测开始
    LoopPairNum=size(cameraPosePair,1);
    if(size(globalCameraPosition,1)>LoopDectNum)
        [cameraPosePair,LoopFlag]=estimateLoop(globalCameraPosition,cameraPosePair,LoopDectNum,maxPairDistance,LoopFlag);
    end
    
    %% 回环结束_特征点匹配_匹配对扩展
    if((LoopPairNum==size(cameraPosePair,1) || i==N) && (LoopFlag==1 ))
        disp('Loop detected completed, Motion Averaging starting...')
        MotionGlobalBackup=MotionGlobal;
        accMotion=fastDesEigMatch(clouds,cameraPosePair,overlap,eigDGridStep,res,MseHold);
%         historyAccMotion=accMotion;
        beforeMotion=(2:length(relativeMotion));
        indiAccMotion=~cellfun(@(a) isempty(a) ,accMotion,'UniformOutput',true);
        trimmedMotion=accMotion(indiAccMotion(:,1),1:3);
        fixMotion=arrayfun(@(x) {relativeMotion{x},x-1,x} , beforeMotion ,'UniformOutput',false ); %补上前面fix结果
        for f=1:length(fixMotion)
            trimmedMotion=[trimmedMotion;fixMotion{f}];
        end
        D=gen_Dij(trimmedMotion,i);
        updatedGlobalMotion=MotionAverage(trimmedMotion,MotionGlobal,D,size(trimmedMotion,1),i);
        for k=1:length(updatedGlobalMotion)
            MotionGlobal{k}=updatedGlobalMotion{k};
        end
        LoopFlag=0;
        lastLoopNum=i;
        maxPairDistance=80;
        LoopDectNum=290;
    end
    
end
generalTime=toc(generalTime)
routeDisplay(MotionGlobalBackup,'b-*',true);
routeDisplay(MotionGlobal,'r-o',false);
% % MotionGlobal=cellfun(@(b) b(1:3,4) ,(cellfun(@(a) inv(a) ,MotionGlobal ,'UniformOutput',false)),'UniformOutput',false);
% load Hannover_GRpose.mat
% pose(:,1)=zeros(length(pose),1);
% routeDisplay(pose,'g-s',true);

obtainResult(clouds,MotionGlobal,false);




