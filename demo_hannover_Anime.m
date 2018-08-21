% ���㷨����������ƥ�䣬ICP���ػ���⣬�˶�ƽ���㷨
clc;clear;close all;

addpath('./flann/');
addpath('./estimateRigidTransform');
eigDGridStep = 30;
% eigLoGridStep=0.03;
mergeGridStep = 5;
readnum=942;
% scannum=length(dir(datapath))-2;
overlap = 0.45;
icpToler= 10;
ICPthreashold= 200;
% %22�ػ������ֵӦ���Ǳ仯��  0.345����ࣩ-12.05maxPairDistance=22;
turnThreshold=0.3;
loopMAmaxNum=18;
turnLengthNum=6;
MseHold=10;
spacebetweenLoop=25;
res= 1;
s= 1;
% clouds=readCloudCsv(filepath,filePrefix,readnum,0.6 ,s);
load hannover2_MZ.mat
load loopDiscCal_poly.mat
generalTime=tic;
N = length(clouds);
MotionGlobal{1}=eye(4);
globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);
LoopDectNum=45;%floor(N/4);
cameraPosePair=[];
turnHead=[];
LoopFlag=0;
turnFlag=0;
lastLoopNum=1;   %�ػ������ֵӦ���Ǳ仯��
currLoop=0;
maxDis=23; %��ʱ[22,100,17,50,245,30]
% fixedPointCloudN={};
historyAccMotion={};
historyCameraPosePair=[];
fullCloud=clouds{1};
%% about the axes
curFig=figure('Position',[7 76 1382 919]);
reAxes=pcshow(fullCloud);
title('Slam On Hannover2')
% reAxes.CameraViewAngleMode = 'auto';
reAxes.CameraPosition=[-3037.32 -4448.29 -5802.49];
reAxes.CameraTarget=[-154.458 346.5108 -457.597];
reAxes.CameraUpVector=[0.35594 0.59201 -0.72306];
reAxes.CameraViewAngle=14.01893;
reScatter = reAxes.Children;

routeFig=figure('Position',[1327.0 49 587.0 922.0]);
routeAxes=axes();
title('Slam Route On Hannover2')
load hannover2_GrtM_z.mat
routeDisplay(GrtM,'g-d',false,[]);
routeAxes.CameraPosition=[165.05 572.619 -9765.724];
routeAxes.CameraTarget=[94.22 416.93 33.024];
routeAxes.CameraUpVector=[0.414 0.9100 0.0174];
routeAxes.CameraViewAngle=8.8671;

pause(6);
%% ����ѭ��
N=903;
i=2;
while i<=N
    disp(['processing frame ' num2str(i) '...']);
    Model=clouds{i-1}.Location';
    Data=clouds{i}.Location';
    R0=relativeMotion{i-1}(1:3,1:3);
    t0=relativeMotion{i-1}(1:3,4);
    [MSE(i,1),R,t] = TrICP(Model, Data, R0, t0, ICPthreashold, overlap);
    relativeMotion{i}=Rt2M(R,t);
    [isTurn , turn(i),planeErr(i)]=turnDectecion(relativeMotion{i},turnThreshold);
    
    if(isTurn && turnFlag==0 )
        turnFlag=turnLengthNum;
        disp(['turn head detected at cloud ' num2str(i)]);
        turnHead=[turnHead,i];
    else
        if(turn(i)<0.11)        %back to straight
            turnFlag=0;
        end
    end
    if(MSE(i,1)>icpToler || turnFlag>0 || planeErr(i)>0.03 )   %��֡��׼������,
        [Motion, mse]=matchFix(clouds{i-1},clouds{i},overlap,eigDGridStep,res,i);
        if(~isempty(Motion))
            relativeMotion{i}=Motion;MSE(i,1)=mse;
        end
        if(MSE(i,1)>30 || norm(relativeMotion{i}(1:3,4))>40)
            relativeMotion{i}=relativeMotion{i-1};
        end
        if(turnFlag>0)
            turnFlag=turnFlag-1;
        end
    end

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
    
    
    %% �ػ���⿪ʼ
    LoopPairNum=size(cameraPosePair,1);
    if(size(globalCameraPosition,1)>LoopDectNum && (i-lastLoopNum>spacebetweenLoop)) %��ֹ��������̫��
        [cameraPosePair,LoopFlag]=estimateLoop(globalCameraPosition,cameraPosePair,LoopDectNum,LoopFlag,lastLoopNum,loopDisCal,maxDis);
    end
    
    %% �ػ�����_������ƥ��_ƥ�����չ
    if((i-lastLoopNum>spacebetweenLoop)&&(LoopPairNum==size(cameraPosePair,1) || LoopPairNum>=loopMAmaxNum-4 || i==N) && (LoopFlag==1 ))
        currLoop=currLoop+1;
        historyCameraPosePair=[historyCameraPosePair;[cameraPosePair,cameraPosePair(:,2)-lastLoopNum]];
        loopNumList(currLoop)=i;
        disp(['Loop ' num2str(currLoop)  ' detected completed, Motion Averaging starting...']);
        if(max(cameraPosePair(:,1))>lastLoopNum+15)     %�ڻ�����
            disp(['Loop ' num2str(currLoop)  ' is inner Loop, storing Loop Motion...']);
            storedMotion=fastDesEigMatch(clouds,cameraPosePair,overlap,eigDGridStep,res,MseHold);
            historyAccMotion=[historyAccMotion;storedMotion];
            disp('Motion Stored!');
            LoopFlag=0;
            cameraPosePair=[];
            continue;
        end
        MotionGlobalBackup=MotionGlobal;
        
        accMotion=fastDesEigMatch(clouds,cameraPosePair,overlap,eigDGridStep,res,MseHold);
        if(isempty(accMotion))  %�������������ƥ�䲢���ã�
            LoopFlag=0;
            cameraPosePair=[];
            lastLoopNum=i;
            disp('wrong loop detected!')
            continue;
        end
        accMotion=[accMotion;historyAccMotion];
        historyAccMotion=accMotion;
        beforeMotion=(2:i);
        indiAccMotion=~cellfun(@(a) isempty(a) ,accMotion,'UniformOutput',true);
        trimmedMotion=accMotion(indiAccMotion(:,1),1:3);
        fixMotion=arrayfun(@(x) {relativeMotion{x},x-1,x} , beforeMotion ,'UniformOutput',false ); %����ǰ��fix���
        fixMotion{length(fixMotion)+1}={eye(4),1,1};
        for f=1:length(fixMotion)
            trimmedMotion=[trimmedMotion;fixMotion{f}];
        end
        D=gen_Dij(trimmedMotion,length(relativeMotion),loopNumList);
        updatedGlobalMotion=MotionAverage(trimmedMotion,MotionGlobal,D,size(trimmedMotion,1),i,2,loopNumList);
        for k=1:length(updatedGlobalMotion)
            MotionGlobal{k}=updatedGlobalMotion{k};
        end
       
        LoopFlag=0;
        cameraPosePair=[];
        %         historyAccMotion={};
        lastLoopNum=i;
        %         maxPairDistance=100;
        %         LoopDectNum=290;
        disp(['Loop ' num2str(currLoop) ', Motion Averaging completed' ] );
        disp(' ');
        %% anime
        cla;routeDisplay(GrtM,'g-d',false,[]);
        routeHandle=routeAnimePlugin(  MotionGlobal,'r-o',routeAxes);
        disp('renew figure...');
        fullCloud=clouds{1};
        for c=2:length(MotionGlobal)
            fullCloud=pcmerge(fullCloud,pcZTransMulti(clouds{c},MotionGlobal{c}),mergeGridStep);
        end
        disp(' ICP forward registering...'  );
    end
    i=i+1;
end
generalTime=toc(generalTime)
routeDisplay(MotionGlobalBackup,'b-*',true,[]);
% turnpoint=abs(turn)>turnThreshold;
turnPoints=[];
for tu=1:length(turnHead)
    turnPoints=[turnPoints,turnHead(tu)-1:turnHead(tu)+turnLengthNum-2];
end
routeDisplay(MotionGlobal,'r-o',false,turnPoints);%443:443+3,462:462+3,491:491+3,495:495+3,526:526+3

routeDisplay(GrtM,'g-d',false,[48,55]);%(1:182)(1:532)799,490

% obtainResult(clouds,MotionGlobal,true);



