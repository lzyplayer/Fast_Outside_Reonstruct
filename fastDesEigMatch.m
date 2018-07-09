function [ motionInfo ] = fastDesEigMatch( clouds, cameraPair,overlap,gridStep,res,MseHold)
%FASTDESEIGMATCH 此处显示有关此函数的摘要
%   此处显示详细说明
% gridStep=0.03;
% overlap=0.35;
% res=10;
eigMSEs=[];motionInfo={};
requiredCloudList=unique(cameraPair(:,1:2));
for i=1:length(requiredCloudList)
    [srcDesp{requiredCloudList(i)},srcSeed{requiredCloudList(i)},srcNorm{requiredCloudList(i)}] = extractEig(clouds{requiredCloudList(i)},gridStep);
end
pairNum=size(cameraPair,1);
for i=1:pairNum
    m=cameraPair(i,1);
    d=cameraPair(i,2);
    Model=clouds{m}.Location(1:res:end,:)';
    Data= clouds{d}.Location(1:res:end,:)';
    T = eigMatch(srcDesp{m},srcDesp{d},srcSeed{m},srcSeed{d},srcNorm{m},srcNorm{d},overlap,gridStep);
    T = inv(T); %为了提高运行效率，所以换过来算
    fixtime=1;
    while(isempty(T)&&fixtime<5)
        [srcDesp{m},srcSeed{m},srcNorm{m}] = extractEig(clouds{m},(1-fixtime/10)*gridStep);
        [srcDesp{d},srcSeed{d},srcNorm{d}] = extractEig(clouds{d},(1-fixtime/10)*gridStep);
        T = eigMatch(srcDesp{m},srcDesp{d},srcSeed{m},srcSeed{d},srcNorm{m},srcNorm{d},0.3,(1-fixtime/10)*gridStep);
        T = inv(T);
        fixtime=fixtime+1;
    end
    if(isempty(T))
        warning(['des cannot fix pair with clouds ' num2str(m) ' ' num2str(d)]);
        R0=[1 0 0; 0 1 0; 0 0 1];
        t0=[0 ;0 ;0];
    else
        disp(['description regis success with clouds: '  num2str(m) ' ' num2str(d)])
    R0= T(1:3,1:3);
    t0= T(1:3,4);
    end
    [MSE,R,t] = TrICP(Model, Data, R0, t0, 100, overlap); 
    %     num= num+1
    if (MSE > MseHold )%5.0*mean(eigMSEs)
        continue;
    end
    eigMSEs= [eigMSEs, MSE];
    accMotion= Rt2M(R,t);
    motionInfo{i,1}=accMotion;
    motionInfo{i,2}=m;
    motionInfo{i,3}=d;
    motionInfo{i,4}=MSE;
end
end