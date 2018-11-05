% 本算法采用特征点匹配，ICP，回环检测，运动平均算法
clc;clear;close all;

addpath('./flann/');
addpath('./estimateRigidTransform');
eigDGridStep = 3;
% eigLoGridStep=0.03;


% scannum=length(dir(datapath))-2;
overlap = 0.45;
icpToler= 0.01;
ICPthreashold= 100;
maxDis=6;           %成为回环匹配对的距离上限
turnThreshold=0.35;
loopMAmaxNum=18;    %回环修复匹配数量临界值
turnLengthNum=5;    %转弯处理数
MseHold=10;
spacebetweenLoop=2;
res= 1;
s= 1;
mergeGridstep=0.03;
load freiburg_rawWithSlamResult.mat
rawClouds=clouds;
clouds=pcTrim(clouds,inf,[0.4 ,inf],0.3);
% pcshow(pcdownsample(clouds{1},'gridAverage',0.3))
LoopDectNum=30;%形成回环至少相差LoopDectNum个序号  %floor(N/4);

generalTime=tic;
N = length(clouds);
MotionGlobal{1}=eye(4);
globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);
cameraPosePair=[];
turnHead=[];
LoopFlag=0;
turnFlag=0;
lastLoopNum=1;   %回环检测阈值应当是变化量
currLoop=0;

% fixedPointCloudN={};
historyAccMotion={};
historyCameraPosePair=[];
%% 　主循环

i=2;
tic
while i<=N
    routeDisplay(MotionGlobal,'r-o',false,[]);
    drawnow
    if (i==6)
        1==1;
    end
    
    disp(['processing frame ' num2str(i) '...']);
    Model=clouds{i-1}.Location';
    Data=clouds{i}.Location';
    R0=relativeMotion{i-1}(1:3,1:3);
    t0=relativeMotion{i-1}(1:3,4);
    [MSE(i,1),R,t] = TrICP(Model, Data, R0, t0, ICPthreashold, overlap);
    relativeMotion{i}=Rt2M(R,t);
    [isTurn , turn(i),~]=turnDectecion(relativeMotion{i},turnThreshold);
    
    if(isTurn && turnFlag==0 )
        turnFlag=turnLengthNum;
        disp(['turn head detected at cloud ' num2str(i)]);
        turnHead=[turnHead,i];
    else
%         if(turn(i)<0.15)        %back to straight
%             turnFlag=0;
%         end
    end
    if(MSE(i,1)>icpToler || turnFlag>0 )   %单帧配准误差过大,
        [Motion, mse]=matchFix(clouds{i-1},clouds{i},overlap,eigDGridStep,res,i);
        if(~isempty(Motion))
            relativeMotion{i}=Motion;MSE(i,1)=mse;
        end
%         if(MSE(i,1)>30 || norm(relativeMotion{i}(1:3,4))>40)
%             relativeMotion{i}=relativeMotion{i-1};
%         end
        if(turnFlag>0)
            turnFlag=turnFlag-1;
        end
    end

    MotionGlobal{i}=MotionGlobal{i-1}*relativeMotion{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
    
    
    
    %% 回环检测开始
    LoopPairNum=size(cameraPosePair,1);
    if(size(globalCameraPosition,1)>LoopDectNum && (i-lastLoopNum>spacebetweenLoop)) %防止两次修正太近
        [cameraPosePair,LoopFlag]=estimateLoop(globalCameraPosition,cameraPosePair,LoopDectNum,LoopFlag,lastLoopNum,maxDis);
    end
    
    %% 回环结束_特征点匹配_匹配对扩展
    if((i-lastLoopNum>spacebetweenLoop)&&(LoopPairNum==size(cameraPosePair,1) || LoopPairNum>=loopMAmaxNum-4 || i==N) && (LoopFlag==1 ))
        currLoop=currLoop+1;
        historyCameraPosePair=[historyCameraPosePair;[cameraPosePair,cameraPosePair(:,2)-lastLoopNum]];
        loopNumList(currLoop)=i;
        disp(['Loop ' num2str(currLoop)  ' detected completed, Motion Averaging starting...']);

        MotionGlobalBackup=MotionGlobal;
        save(['Loop' num2str(currLoop) 'BeforeMA'],'MotionGlobalBackup');
        accMotion=fastDesEigMatch(clouds,cameraPosePair,overlap,eigDGridStep,res,MseHold);
        if(isempty(accMotion))  %估测错误，特征点匹配并不好，
            LoopFlag=0;
            cameraPosePair=[];
            lastLoopNum=i;
            warning('wrong loop detected!')
            continue;
        end
        accMotion=[accMotion;historyAccMotion];
        historyAccMotion=accMotion;
        beforeMotion=(2:i);
        indiAccMotion=~cellfun(@(a) isempty(a) ,accMotion,'UniformOutput',true);
        trimmedMotion=accMotion(indiAccMotion(:,1),1:3);
        fixMotion=arrayfun(@(x) {relativeMotion{x},x-1,x} , beforeMotion ,'UniformOutput',false ); %补上前面fix结果
        fixMotion{length(fixMotion)+1}={eye(4),1,1};
        for f=1:length(fixMotion)
            trimmedMotion=[trimmedMotion;fixMotion{f}];
        end
        D=gen_Dij(trimmedMotion,length(relativeMotion),loopNumList);
        updatedGlobalMotion=MotionAverage(trimmedMotion,MotionGlobal,D,size(trimmedMotion,1),i,2,loopNumList);
        for k=1:length(updatedGlobalMotion)
            MotionGlobal{k}=updatedGlobalMotion{k};
        end
        save(['Loop' num2str(currLoop)],'MotionGlobal');
        LoopFlag=0;
        cameraPosePair=[];
        %         historyAccMotion={};
        lastLoopNum=i;
        %         maxPairDistance=100;
        %         LoopDectNum=290;
        disp(['Loop ' num2str(currLoop) ', Motion Averaging completed' ] );
        disp(' ');
        disp(' ICP forward registering...'  );
    end
    i=i+1;
end
toc
generalTime=toc(generalTime)
routeDisplay(MotionGlobalBackup,'b-*',true,[]);
% turnpoint=abs(turn)>turnThreshold;
turnPoints=[];
for tu=1:length(turnHead)
    turnPoints=[turnPoints,turnHead(tu)-1:turnHead(tu)+turnLengthNum-2];
end
routeDisplay(MotionGlobal,'r-o',false,[]);%443:443+3,462:462+3,491:491+3,495:495+3,526:526+3

routeDisplay(gslamResult,'g-d',false,[]);%

obtainResult(clouds,MotionGlobal,true,mergeGridstep);



