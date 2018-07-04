clc
clear all
close all
%% the initial parameters
global TrMin TrMax N
TrMin= 0.35;
TrMax= 1.0;
Trim= 0.5; % accepting threshhold
res= 10;
lamda= 2;
iterationThreshlod= 30;
% load bunny;
load dragon;
% load dragon;
% load RotTranH;
% load dragon; 
% load RotTranD;
% load ChefData;

shape= dragon;
RotTran= RotTranD;
RotTran(:,1:3) = RotTran (:,1:3)+0.07;
tic
scannum=length(shape);
scan=shape(:,1);     
for i=1:scannum
    scan{i}=1000*scan{i};
end 
Motion=initialiseM(scannum,RotTran);
% for i=1:scannum
%     theta = 0.1*(2*rand(3,1)-1);
%     Motion{i}(1:3,1:3) = OulerToRota(theta)*Motion{i}(1:3,1:3);
%     % compute the nearest rotation matrix
%     [u,~,v]=svd(Motion{i}(1:3,1:3));
%     Motion{i}(1:3,1:3)=u*diag([1,1,det(u*v')])*v';
% end

%% get initial registration
[scan,Mshape]=obtainShape( scan,Motion );
%crosssection(Mshape,1,-20.1,-19.8);%bunny
%% build the kd tree
for i=1:scannum
    model=scan{i}(1:3,1:res:end);
    ns{i}=createns(model','nsmethod','kdtree');
end
%% MAICP
iter= 0;    err= 10;
ERROR= (length(scan)-1)*90*10^(-5);
while ((iter<iterationThreshlod)&(err>(ERROR)))
    iter= iter+1;
    D= [];   num= 0;  err= 0; indexi=[]; indexj=[];
    %% calculate  overlap rate
    if (iter<5)
        overlapRate=overlapRateEveluation(scan,Motion,res,TrMin,TrMax,lamda, ns);
    end
    for i=1: scannum
        for j= 1: scannum
            if((i~=j)&(overlapRate(i,j)>Trim))
                num= num+1;
                updatedMotion{num,2}=i;
                updatedMotion{num,3}=j;
                Dij= gen_Dij(i,j,length(scan));
                D= [D;Dij];
            end
        end
    end
    parfor k=1:num
        i=updatedMotion{k,2};
        j=updatedMotion{k,3};
        relativeMotion=inv(Motion{i})*Motion{j};
        Model=scan{i}(:,1:res:end);
        Data=scan{j}(:,1:res:end);
        updatedMotionM{k}= TrimmedICP(ns,Model, Data,relativeMotion,i,iterationThreshlod,TrMin,TrMax,lamda);
    end
    for k=1:num
        updatedMotion{k,1}=updatedMotionM{k};
    end
    preMotion= Motion;
    Motion= MotionAverage(updatedMotion,Motion,D,num,length(scan));
    err=comErr(preMotion,Motion,length(scan));
end
Runtime = toc
ObjV= com_objv( Motion, ns, scan,res)/ scannum
[scan,Mshape]=obtainShape( scan,Motion );
% crosssection(Mshape,1,-20.1,-19.8);                 % bunny
% crosssection(Mshape,2,149.8,150.2);                        % Happy
crosssection(Mshape,2,99.8,100.2);                         % Dragon
% crosssection(Model,2,0.2,0.8);   % Chicken
%crosssection(Model,1,0.2,0.8);   % Para
% crosssection(Mshape,1,54.2,54.8);   % Chef
% crosssection(Model,1,-2.8,-2.2);   % Trex
%     Data.vertex.x = [Mshape(1,:)];
%     Data.vertex.y = [Mshape(2,:)];
%     Data.vertex.z = [Mshape(3,:)];
%     ply_write(Data,['./bunny.ply'],'ascii');
%%



