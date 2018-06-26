% 本算法
clc;clear;close all;

addpath('./flann/');
addpath('./estimateRigidTransform');
icpGridStep = 0.01;
eigDGridStep = 0.04;
eigLoGridStep=0.03;
filepath='./data/local_frame/';
filePrefix='Hokuyo_';
readnum=31;
% scannum=length(dir(datapath))-2;
overlap = 0.35;
icpToler=2e-05;
ICPthreashold=50;
res= 10;
s=30;
% [clouds,Desp,Seed,Norm] = readRoom(data,gridStep);
% [clouds] = readRawOutside(filepath,filePrefix,readnum,s);
% [clouds,Desp,Seed,Norm] = readOutside(filepath,filePrefix,readnum,gridStep,s);
load trimOutside.mat
tic;
N = length(clouds);
MotionGlobal{1}=eye(4);
dowmSampleClouds{1}=pcdownsample(pcdenoise(clouds{1}),'gridAverage',icpGridStep);
% ns{1}=createns(dowmSampleClouds{1}.Location,'nsmethod','kdtree');
globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);
LoopDectNum=ceil(N/2);
cameraPosePair=[];
LoopFlag=0;
%% 　主循环
for i=2:N
    dowmSampleClouds{i}=pcdownsample(pcdenoise(clouds{i}),'gridAverage',icpGridStep);
%     [currMotion2next]=pcregrigid(dowmSampleClouds{i-1},dowmSampleClouds{i},'Tolerance',[0.01/s,0.009]);
    ns{i-1}=createns(dowmSampleClouds{i-1}.Location,'nsmethod','kdtree');
    IcpModel=[dowmSampleClouds{i-1}.Location';ones(1,dowmSampleClouds{i-1}.Count)];
    IcpData=[dowmSampleClouds{i}.Location';ones(1,dowmSampleClouds{i}.Count)];
    [relativeMotion{i},MSE(i,1)]=myTrimICP(ns{i-1},IcpModel,IcpData,relativeMotion{i-1},ICPthreashold,overlap);
%     fixTimes=0; enhanceModelCloud=dowmSampleClouds{i-1};
    if(MSE(i,1)>icpToler )   %单帧配准误差过大,
          [relativeMotion{i}, MSE(i,1)]=matchFix(clouds{i-1},clouds{i},overlap,eigDGridStep,res);
    end
    MotionGlobal{i}=MotionGlobal{i-1}*relativeMotion{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
    %% 回环检测开始
    LoopPairNum=size(cameraPosePair,1);
    if(size(globalCameraPosition,1)>LoopDectNum)
        cameraPosePair=estimateLoop(globalCameraPosition,cameraPosePair,LoopDectNum);
        
    end
    %% 回环结束_特征点匹配_匹配对扩展
    if((LoopPairNum==size(cameraPosePair,1)&& size(cameraPosePair,1)>0  && LoopFlag==0)||(LoopFlag==0 && i==N ))
%         accMotion=fastDesEigMatch(clouds,cameraPosePair,overlap,eigLoGridStep,res);
        LoopFlag=1;
    end
        
end
toc
% N = length(clouds);
% p(1).M = eye(4);   
% tarCloud = clouds{1};
% tarDesp = Desp{1};
% tarSeed = Seed{1};
% tarNorm = Norm{1};
% Model= tarCloud.Location(1:res:end,:)'*s;
% id = 2:N;
% idmatch = [];
% num= 0;
% % tic;
% MSEs= 0.8*cal_mMSEs(Model);
% while(~isempty(id))
%     size = length(id);
%     for i = id 
%         srcCloud = clouds{i};
%         srcDesp = Desp{i};
%         srcSeed = Seed{i};
%         srcNorm = Norm{i};
%         Data= srcCloud.Location(1:res:end,:)'*s;
% %       T = eigMatch(srcDesp,tarDesp,srcSeed,tarSeed,srcNorm,tarNorm,overlap,gridStep);
%         T = eigMatch(tarDesp,srcDesp,tarSeed,srcSeed,tarNorm,srcNorm,overlap,gridStep);
%         T = inv(T); %为了提高运行效率，所以换过来算
%         R0= T(1:3,1:3);
%         t0= T(1:3,4)*s;
%         TData = transform_to_global(Data, R0, t0);
% %         dM= cal_mMSEs(Model);
%         [MSE,R,t,TData,PCorr, Dthr] = TrICP(Model, Data, R0, t0, 100, overlap); %这个操作特别费时间
%         num= num+1
%         if (MSE > 2.0*mean(MSEs))
%             continue;
%         end
%         MSEs= [MSEs, MSE];      
%         p(i).M = Rt2M(R,t);
%         srcSeed = transform_to_global(srcSeed,R,t/s);
%         FCorr= Fcorr(tarSeed,srcSeed,Dthr/s);
%         tarSeed= merge(tarSeed, srcSeed, FCorr);
%         tarDesp= merge(tarDesp, srcDesp, FCorr);
%         srcNorm= transform_norm(srcNorm,R);
%         tarNorm= merge(tarNorm, srcNorm, FCorr);
%         Model= merge(Model, TData, PCorr);
% %         figure;
% %         plot3(Model(1,1:res:end),Model(2,1:res:end),Model(3,1:res:end),'.','Color','r');
% %         hold on;
% %         plot3(TData(1,1:res:end),TData(2,1:res:end),TData(3,1:res:end),'.','Color','g');
%         id = setdiff(id,i);
%         idmatch = [idmatch i]
%     end
%     if(size == length(id))
%         disp('the mutil-View process fails!')
%         break;
%     end
% end
% Time = toc/60
% % shape=cell(N,1);
% % for i = 1:N 
% %     shape{i,1}= clouds{i}.Location;
% % end
% % for i = 1:N
% %     colorInfo{i}=clouds{i}.Color;
% % %     colorInfo{i}=data{i}(:,4:6);
% % end
% figure;
% 
% Model=obtain_room_colorful(clouds, p, N, s);