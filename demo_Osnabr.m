% ���㷨����������ƥ�䣬ICP���ػ���⣬�˶�ƽ���㷨
clc;close all;

addpath('./flann/');
addpath('./estimateRigidTransform');
% load AVZbuilding_ori.mat  ;
icpGridStep = 9;
eigDGridStep = 0.3; %����gridStepEstimate�õ���ֵ
readnum=63;             %64 in total , start from 0
% scannum=length(dir(datapath))-2;
overlap = 0.3;
icpToler= 2;
ICPthreashold= 50;
maxPairDistance=800;
res= 1;
s= 1;

% ׼����׼֡
N = length(clouds);
MotionGlobal{1}=eye(4);
globalCameraPosition=[0,0,0];
relativeMotion{1}=eye(4);
LoopDectNum=floor(N/4);
cameraPosePair=[];
LoopFlag=0;
ns={};
fixedPointCloudN={};


% ��ѭ��
generalTime=tic;
for i=2
    

%     Model= clouds{i-1}.Location(1:res:end,:)';
%     Data= clouds{i}.Location(1:res:end,:)';
%     R0=relativeMotion{i-1}(1:3,1:3); t0=relativeMotion{i-1}(1:3,4);
%     [MSE(i,1),R,t] = TrICP(Model, Data, R0, t0, ICPthreashold, overlap);
%     relativeMotion{i} = Rt2M(R,t);

%     if(MSE(i,1)>icpToler  )  %��֡��׼������,   
          [relativeMotion{i}, MSE(i,1)]=matchFix(clouds{i-1},clouds{i},overlap,eigDGridStep,1);
%     end
    MotionGlobal{i}=MotionGlobal{i-1}*relativeMotion{i};
    globalCameraPosition(i,:)=MotionGlobal{i}(1:3,4)';
    
%     routeDisplay(MotionGlobal,'b-o',false);
    
    
    %% �ػ���⿪ʼ
    LoopPairNum=size(cameraPosePair,1);
    if(size(globalCameraPosition,1)>LoopDectNum)
        [cameraPosePair,LoopFlag]=estimateLoop(globalCameraPosition,cameraPosePair,LoopDectNum,maxPairDistance,LoopFlag);
    end
    %% �ػ�����_������ƥ��_ƥ�����չ
    if((LoopPairNum==size(cameraPosePair,1) || i==N) && (LoopFlag==1 ))
        routeDisplay(MotionGlobal,'b-*',true);
        accMotion=fastDesEigMatch(clouds,cameraPosePair,overlap,eigDGridStep,res);
        beforeMotion=(2:length(relativeMotion));
        fixMotion=arrayfun(@(x) {relativeMotion{x},x-1,x} , beforeMotion ,'UniformOutput',false ); %����ǰ��fix���
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

routeDisplay(MotionGlobal,'r-o',false);
load Osnabr_Grt.mat;

routeDisplay(GrtM,'g-s',false);

% obtainResult(clouds,MotionGlobal);


