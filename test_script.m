clc;clear;close all;
%% 真值处理
load hannover_GrtM_z_ConvertNeed.mat
% load hannover2_GrtM_ori.mat
load hannover2_result.mat
load hannover2_MZ.mat
% 重组为矩阵
% for i=1:size(GrtM_ori,1)
%     GrtM_ori_m{i}=reshape( GrtM_ori(i,:),4,4);
% end
% 
nums=length(GrtM);
for i=1:nums
    GrtM{i}=(GrtM{1}) \ GrtM{i};
end
mirrorM=[1 0 0 0;
         0 1 0 0;
         0 -1 1 0;
         0 0 0 1];

% GrtM_RelativeTo1=cell(nums,1);
GrtMatrixRightHand=cellfun(@(x) mirrorM*x,GrtM,'UniformOutput',0);

% %%     new
%     tfrom=MotionGlobal{47}*inv((GrtM_RelativeTo1{47}));
%     rot=tfrom(1:3,1:3);
for i=1:903
%     newMotion{i}=[rot*GrtM_RelativeTo1{i}(1:3,1:3),MotionGlobal{i}(1:3,4);0 0 0 1];
    GrtMatrixRightHand{i}(1:3,1:3)*GrtMatrixRightHand{i}(1:3,1:3)'
end
% obtain_model_hannover1(clouds,newMotion);
%     
%%
for i=1:length(GrtM_RelativeTo1)
    %% new
    resu{i}=rotm2quat(GrtM_RelativeTo1{i}(1:3,1:3));
    resu1{i}=rotm2quat(MotionGlobal{i}(1:3,1:3));
%% old
    EulerIn{i}=rotm2eul(GrtM_RelativeTo1{i}(1:3,1:3),'XYZ');
    EulerIn2{i}=rotm2eul(GrtM_RelativeTo1{i}(1:3,1:3),'ZYX');
    EulerNeed{i}=rotm2eul(MotionGlobal{i}(1:3,1:3),'XYZ');
    conEuler=[EulerIn{i}(3),EulerIn{i}(1),EulerIn{i}(2)];
    conRot=eul2rotm(conEuler,'XYZ');
    conTran=[GrtM_RelativeTo1{i}(3,4);GrtM_RelativeTo1{i}(1,4);GrtM_RelativeTo1{i}(2,4)];
    conGrtM{i}=Rt2M(conRot,conTran);
     EulerIn{i}=rotm2eul(conGrtM{i}(1:3,1:3),'ZYZ');

end
% obtain_model_hannover1(clouds,newMotion);

%% 真值环境展示
load hannover2_MZ.mat
load hannover2_GrtM_z.mat
routeDisplay(GrtM,'g-d',false,[48,55]);%(1:182)(1:532)799,490
obtainResult(clouds,GrtM(1:100),true);


%% 相对位置变化限制
for i=2:length(relativeMotion)
    distance(i)=norm(  relativeMotion{i}(1:3,4));
end
%% 展示historyAccMotion中一对
figure;
toseeACC=8;
low=historyAccMotion{toseeACC,2};
high=historyAccMotion{toseeACC,3};
pcshow(clouds{low});hold on;
pcshow(pctransform(clouds{high},affine3d(historyAccMotion{toseeACC}')));%historyAccMotion{pairnum}'

%% 展示相对运动某对
tosee=2;
% for currsee=tosee:tosee+8
figure;
pcshow(clouds{tosee-1});hold on;
pcshow(pctransform(clouds{tosee},affine3d(relativeMotion{tosee}')));%historyAccMotion{pairnum}'
% end
%% 分别展示2帧
figure;
a=6;
pcshow(pctransform( clouds{a},affine3d(MotionGlobal{a}')));
hold on;
% figure;
b=7;
pcshow(pctransform( clouds{b},affine3d(MotionGlobal{b}')));

% pc自带ICp算法
gridstep=0.01;
readnum=31;
overlap = 0.4;
res= 1;
s=30;
filepath='./data/local_frame/';
filePrefix='Hokuyo_';
load trimOutside;
% 自带配准效果不好
readyCloud1=pcdownsample(pcdenoise(clouds{4}),'gridAverage',gridstep);
readyCloud2=pcdownsample(pcdenoise(clouds{3}),'gridAverage',gridstep);
[motion,result]=pcregrigid(readyCloud2,readyCloud1,'Tolerance',[0.01/s,0.009]);
pcshow(result);
hold on;

% 相邻展现
pcshow(clouds{4});
hold on;
pcshow(pctransform(clouds{3},currMotion2next));

%我们ICP，加利用上回运动
d=13;
m=14;
readyCloud1=pcdownsample(pcdenoise(clouds{d}),'gridAverage',gridstep);
readyCloud2=pcdownsample(pcdenoise(clouds{m}),'gridAverage',gridstep);
ns=createns(readyCloud2.Location,'nsmethod','kdtree');
motion=myTrimICP(ns,[readyCloud2.Location';ones(1,readyCloud2.Count)],[readyCloud1.Location';ones(1,readyCloud1.Count)],relativeMotion{m-1},50,0.35);
pcshow(clouds{m});
hold on;
pcshow(pctransform(clouds{d},affine3d(motion')));

%% 全局方向展现，和路径结合用
% ori=[0 0 0 ;100 100 100 ];%全局观测方向
ori=[0 0 0 ;0.01 0.01 0.01 ];

for i=1:length(MotionGlobal)
    currOri=MotionGlobal{i}*[ori';ones(1,size(ori,1))];
    eachOri{i}=currOri(1:3,:)';
    plot3(eachOri{i}(:,1),eachOri{i}(:,2),eachOri{i}(:,3),'r-o')
    hold on
end

%% 展示路径图

route=[];
for p=1:length(MotionGlobal)
    route=[route; MotionGlobal{p}(1:3,4)'];
end
plot3(route(:,1),route(:,2),route(:,3),'-*');
xlabel('x');
ylabel('y');
zlabel('z');
% axis([-3000 3000 -3000 3000 -3000 3000 ]);
axis([-0.2 0.2 -0.2 0.2 -0.2 0.2 ]);

% %test德国室内点云
% scanTest=load('./data/scanTest.3d');
% cloudTest=pointCloud(scanTest(:,1:3));
% pcshow(cloudTest);
% 


%% 某两帧eig匹配
s=1;
ModelCloud=clouds{6};
DataCloud=clouds{7};
% ModelCloud=pointCloud(pointCMap1.Location);%clouds{234}
% DataCloud=pointCloud(pointCMap2.Location);%clouds{687}
gridStep= 6;%30
overlap=0.4;
res=1;
tic
[tarDesp,tarSeed,tarNorm] = extractEig(ModelCloud,gridStep); 
[srcDesp,srcSeed,srcNorm] = extractEig(DataCloud,gridStep);
T = eigMatch(tarDesp,srcDesp,tarSeed,srcSeed,tarNorm,srcNorm,overlap,gridStep);
T = inv(T);
R0= T(1:3,1:3);
t0= T(1:3,4);
Model= ModelCloud.Location(1:res:end,:)';
Data= DataCloud.Location(1:res:end,:)';

[MSE,R,t] = TrICP(Model, Data, R0, t0, 100, overlap);
Motion=Rt2M(R,t);
Motion(1:3,4)=Motion(1:3,4).*s;
toc
pcshow(ModelCloud);hold on
pcshow(pcZTransMulti(DataCloud,Motion));

return
% 
%展示某帧和第一帧配准
close all;
tar=3;
pcshow(clouds{1});
hold on;
pcshow(pctransform(clouds{tar},affine3d( Motion')));


% %% 某帧真值比对 
% load outside_GRT;
% tar =28;
% src=1;
% realMotion=GrtM{src}\GrtM{tar};   % inv(GrtM{src})*GrtM{tar}
% norm(realMotion(1:3,1:3)-motionInfo{1,1}(1:3,1:3),'fro')
% norm(realMotion(1:3,4)-motionInfo{1,1}(1:3,4).*30,2)
% 
% 

%% test CEll_fun
fixMotion=cellfun(@(x) {relativeMotion{x},x-1,x} , {2,3},'UniformOutput',false )
cellfun(@(x) {relativeMotion{x},x-1,x} , fixedPointCloudN,'UniformOutput',false );
cell2mat(fixMotion)
[historyAccMotion;fixMotion{1}]

%% 求两点距离
p1=[86.53 -21.02]
p2=[80.86 -20.12]
norm(p2-p1)

%% 点间关系拟合
x=historyCameraPosePair(:,2)-historyCameraPosePair(:,1);
y=historyCameraPosePair(:,4);
z=historyCameraPosePair(:,3);
reTool1=cftool(x,y,z+10);