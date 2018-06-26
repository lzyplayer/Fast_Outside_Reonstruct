gridstep=0.01;
% readnum=31;
% overlap = 0.4;
% res= 1;
% s=30;
% filepath='./data/local_frame/';
% filePrefix='Hokuyo_';
% load trimOutside;
% 自带配准效果不好
% readyCloud1=pcdownsample(pcdenoise(clouds{4}),'gridAverage',gridstep);
% readyCloud2=pcdownsample(pcdenoise(clouds{3}),'gridAverage',gridstep);
% [motion,result]=pcregrigid(readyCloud2,readyCloud1,'Tolerance',[0.01/s,0.009]);
% pcshow(result);
% hold on;
% 
% 相邻展现
% pcshow(clouds{4});
% hold on;
% pcshow(pctransform(clouds{3},currMotion2next));

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

%展示路径图
 plot(globalCameraPosition(:,1),globalCameraPosition(:,2),'r-*');
xlabel('x');
ylabel('y');
% zlabel('z');
axis([-0.2 0.2 -0.2 0.2 ]);

newposition=[];
for p=1:30
    newposition=[newposition; MotionGlobal{p}(1:3,4)'];
end



%% 某两帧eig匹配
tic
ModelCloud=clouds{1};
DataCloud=clouds{28};
gridStep=0.03;
overlap=0.35;
res=10;
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
Motion(1:3,4)=Motion(1:3,4);
toc


%展示某帧和第一帧配准
close all;
tar=13;
pcshow(clouds{tar});
hold on;
pcshow(pctransform(clouds{1},affine3d( MotionGlobal{tar}')));


%% 某帧真值比对 
load outside_GRT;
tar =28;
src=1;
realMotion=GrtM{src}\GrtM{tar};   % inv(GrtM{src})*GrtM{tar}
norm(realMotion(1:3,1:3)-motionInfo{1,1}(1:3,1:3),'fro')
norm(realMotion(1:3,4)-motionInfo{1,1}(1:3,4).*30,2)


