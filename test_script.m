gridstep=0.01;
% readnum=31;
% overlap = 0.4;
% res= 1;
% s=30;
% filepath='./data/local_frame/';
% filePrefix='Hokuyo_';
% load trimOutside;
% �Դ���׼Ч������
% readyCloud1=pcdownsample(pcdenoise(clouds{4}),'gridAverage',gridstep);
% readyCloud2=pcdownsample(pcdenoise(clouds{3}),'gridAverage',gridstep);
% [motion,result]=pcregrigid(readyCloud2,readyCloud1,'Tolerance',[0.01/s,0.009]);
% pcshow(result);
% hold on;
% 
% ����չ��
% pcshow(clouds{4});
% hold on;
% pcshow(pctransform(clouds{3},currMotion2next));

%����ICP���������ϻ��˶�
d=13;
m=14;
readyCloud1=pcdownsample(pcdenoise(clouds{d}),'gridAverage',gridstep);
readyCloud2=pcdownsample(pcdenoise(clouds{m}),'gridAverage',gridstep);
ns=createns(readyCloud2.Location,'nsmethod','kdtree');
motion=myTrimICP(ns,[readyCloud2.Location';ones(1,readyCloud2.Count)],[readyCloud1.Location';ones(1,readyCloud1.Count)],relativeMotion{m-1},50,0.35);
pcshow(clouds{m});
hold on;
pcshow(pctransform(clouds{d},affine3d(motion')));

%չʾ·��ͼ
 plot(globalCameraPosition(:,1),globalCameraPosition(:,2),'-*');
xlabel('x');
ylabel('y');
% zlabel('z');
axis([-0.5 0.5 -0.5 0.5 ]);

%չʾĳ֡�͵�һ֡��׼
close all;
tar=13;
pcshow(clouds{tar});
hold on;
pcshow(pctransform(clouds{1},affine3d( MotionGlobal{tar}')));


% 
