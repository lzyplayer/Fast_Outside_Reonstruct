function [ Motion ,MSE ] = matchFix( ModelCloud,DataCloud ,overlap,gridStep,res)
%MATCHFIX 此处显示有关此函数的摘要
%   此处显示详细说明
tic
% gridStep=0.04;
% overlap=0.35;
% res=10;
[tarDesp,tarSeed,tarNorm] = extractEig(ModelCloud,gridStep);
[srcDesp,srcSeed,srcNorm] = extractEig(DataCloud,gridStep);
T = eigMatch(tarDesp,srcDesp,tarSeed,srcSeed,tarNorm,srcNorm,overlap,gridStep);
T = inv(T);
R0= T(1:3,1:3);
t0= T(1:3,4);
Model= ModelCloud.Location(1:res:end,:)';
Data= DataCloud.Location(1:res:end,:)';
% TData = transform_to_global(Data, R0, t0);
[MSE,R,t] = TrICP(Model, Data, R0, t0, 100, overlap);
Motion=Rt2M(R,t);
Motion(1:3,4)=Motion(1:3,4);
fixtime=toc;
disp(['fixtime is ' num2str(fixtime) 'seconds']);
end

