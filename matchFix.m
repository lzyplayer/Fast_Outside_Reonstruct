function [ Motion ,MSE ] = matchFix( ModelCloud,DataCloud ,overlap,gridStep,res)
%MATCHFIX 此处显示有关此函数的摘要
%   此处显示详细说明
tic
% gridStep=0.04;
% overlap=0.35;
% res=10;
MSE=0;
[tarDesp,tarSeed,tarNorm] = extractEig(ModelCloud,gridStep);
[srcDesp,srcSeed,srcNorm] = extractEig(DataCloud,gridStep);
if(size(tarDesp,2)<100 || size(srcDesp,2)<100)
    warning(['description points not enough! ' num2str(size(tarDesp,2)) ' ' num2str(size(srcDesp,2))])
end
if(size(tarDesp,2)<size(srcDesp,2))
    T = eigMatch(srcDesp,tarDesp,srcSeed,tarSeed,srcNorm,tarNorm,overlap,gridStep);
else
    T = eigMatch(tarDesp,srcDesp,tarSeed,srcSeed,tarNorm,srcNorm,overlap,gridStep);
    T = inv(T);
end
fixtime=1;
while(isempty(T)&&fixtime<4)
    [tarDesp,tarSeed,tarNorm] = extractEig(ModelCloud,(1-fixtime/10)*gridStep);
    [srcDesp,srcSeed,srcNorm] = extractEig(DataCloud,(1-fixtime/10)*gridStep);
    T = eigMatch(tarDesp,srcDesp,tarSeed,srcSeed,tarNorm,srcNorm,overlap,(1-fixtime/10)*gridStep);
    T = inv(T);
    fixtime=fixtime+1;
    
end
if (isempty(T))
    Motion=[];
    return ;
end
R0= T(1:3,1:3);
t0= T(1:3,4);
Model= ModelCloud.Location(1:res:end,:)';
Data= DataCloud.Location(1:res:end,:)';
%
[MSE,R,t] = TrICP(Model, Data, R0, t0, 100, overlap);
Motion=Rt2M(R,t);
% Motion=Rt2M(R0,t0);
fixtime=toc;
disp(['fixtime is ' num2str(fixtime) 'seconds']);

end

