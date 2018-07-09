% ���㷨����������ƥ�䣬ICP���ػ���⣬�˶�ƽ���㷨
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
maxPairDistance=22;%22�ػ������ֵӦ���Ǳ仯��  0.345����ࣩ-12.05
loopMAmaxNum=15;
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
LoopDectNum=50;%floor(N/4);
cameraPosePair=[];
LoopFlag=0;
lastLoopNum=1;   %�ػ������ֵӦ���Ǳ仯��
currLoop=0;
% fixedPointCloudN={};
historyAccMotion={};
%% ����ѭ��
N=670;
for i=2:N
%     if i==530
%         0==0;
%     end
    Model=clouds{i-1}.Location';
    Data=clouds{i}.Location';
    R0=relativeMotion{i-1}(1:3,1:3);
    t0=relativeMotion{i-1}(1:3,4);
    [MSE(i,1),R,t] = TrICP(Model, Data, R0, t0, ICPthreashold, overlap);
    relativeMotion{i}=Rt2M(R,t);
    if(MSE(i,1)>icpToler || i==239 || ((i>680)&&(i<689)) )   %��֡��׼������,     
        [Motion, mse]=matchFix(clouds{i-1},clouds{i},overlap,eigDGridStep,res,i);
        if(~isempty(Motion))
            relativeMotion{i}=Motion;MSE(i,1)=mse;
        end
    end
    MotionGlobal{i}=MotionGlobal{i-1}*relativeMotion{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
    
    
    
    %% �ػ���⿪ʼ
    LoopPairNum=size(cameraPosePair,1);
    if(size(globalCameraPosition,1)>LoopDectNum && (i-lastLoopNum>35)) %��ֹ��������̫��
        [cameraPosePair,LoopFlag]=estimateLoop(globalCameraPosition,cameraPosePair,LoopDectNum,maxPairDistance,LoopFlag);
    end
    
    %% �ػ�����_������ƥ��_ƥ�����չ
    if((i-lastLoopNum>35)&&(LoopPairNum==size(cameraPosePair,1) || LoopPairNum>=loopMAmaxNum-3 || i==N) && (LoopFlag==1 ))
        currLoop=currLoop+1;
        disp(['Loop ' num2str(currLoop)  ' detected completed, Motion Averaging starting...'])
        MotionGlobalBackup=MotionGlobal;
        accMotion=fastDesEigMatch(clouds,cameraPosePair,overlap,eigDGridStep,res,MseHold);
        if(isempty(accMotion))  %�������������ƥ�䲢���ã�
            LoopFlag=0;
            cameraPosePair=[];
            lastLoopNum=i;
            continue;
        end
        accMotion=[accMotion;historyAccMotion];
        historyAccMotion=accMotion;
        beforeMotion=(2:length(relativeMotion));
        indiAccMotion=~cellfun(@(a) isempty(a) ,accMotion,'UniformOutput',true);
        trimmedMotion=accMotion(indiAccMotion(:,1),1:3);
        fixMotion=arrayfun(@(x) {relativeMotion{x},x-1,x} , beforeMotion ,'UniformOutput',false ); %����ǰ��fix���
        fixMotion{length(fixMotion)+1}={eye(4),1,1};
        for f=1:length(fixMotion)
            trimmedMotion=[trimmedMotion;fixMotion{f}];
        end
        D=gen_Dij(trimmedMotion,i);
        updatedGlobalMotion=MotionAverage(trimmedMotion,MotionGlobal,D,size(trimmedMotion,1),i);
        for k=1:length(updatedGlobalMotion)
            MotionGlobal{k}=updatedGlobalMotion{k};
        end
        LoopFlag=0;
        cameraPosePair=[];
        lastLoopNum=i;
%         maxPairDistance=100;
%         LoopDectNum=290;
        disp(['Loop ' num2str(currLoop) ', Motion Averaging completed' ] );
        disp(' ');
        disp(' ICP forward registering'  );
    end
    
end
generalTime=toc(generalTime)
routeDisplay(MotionGlobalBackup,'b-*',true);
routeDisplay(MotionGlobal,'r-o',false);

% load hannover_GrtM_z_ConvertNeed.mat
% routeDisplay(GrtM(1:680),'g-d',true);%(1:532)

obtainResult(clouds,MotionGlobal,false);




