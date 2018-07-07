% 本算法采用特征点匹配，ICP，回环检测，运动平均算法
clc;clear;close all;

addpath('./flann/');
addpath('./estimateRigidTransform');
eigDGridStep = 42.7188;
% eigLoGridStep=0.03;

readnum=942;
% scannum=length(dir(datapath))-2;
overlap = 0.35;
icpToler= 3;
ICPthreashold= 200;
maxPairDistance=1.2;
res= 1;
s= 1;
% clouds=readCloudCsv(filepath,filePrefix,readnum,0.6 ,s);
load hannover2zoomed.mat;

generalTime=tic;
N = length(clouds);
MotionGlobal{1}=eye(4);
globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);
LoopDectNum=10;%floor(N/4);
cameraPosePair=[];
LoopFlag=0;
ns={};
fixedPointCloudN={};
%% 　主循环
for i=2:50
    Model=clouds{i-1}.Location';
    Data=clouds{i}.Location';
    R0=relativeMotion{i-1}(1:3,1:3);
    t0=relativeMotion{i-1}(1:3,4);
    [MSE(i,1),R,t] = TrICP(Model, Data, R0, t0, ICPthreashold, overlap);
    relativeMotion{i}=Rt2M(R,t);
    if(MSE(i,1)>icpToler )   %单帧配准误差过大,
        [Motion, mse]=matchFix(clouds{i-1},clouds{i},overlap,eigDGridStep,res);
        if(~isempty(Motion))
            relativeMotion{i}=Motion;MSE(i,1)=mse;
        end
    end
    MotionGlobal{i}=MotionGlobal{i-1}*relativeMotion{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
    
    
end
obtainResult(clouds,MotionGlobal);
routeDisplay(MotionGlobal,'r-o',false);