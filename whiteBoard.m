clc;clear;close all ;
addpath('./flann/');
addpath('./estimateRigidTransform');
s=100;
% ModelCloud=clouds{1};
% DataCloud=clouds{2};
load 3dtry.mat;
tic
ModelCloud=pointCloud(pointCMap1.Location./s);%clouds{234}
DataCloud=pointCloud(pointCMap2.Location./s);%clouds{687}
gridStep= 0.3;%30
overlap=0.2;
res=1;
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

pcshow(pointCMap1);
hold on;
pcshow(pctransform(pointCMap2,affine3d( Motion')));
colormap([0,0,0]);
