function [ motionInfo ] = fastDesEigMatch( clouds, cameraPair,overlap,gridStep,res)
%FASTDESEIGMATCH �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
% gridStep=0.03;
% overlap=0.35;
% res=10;
eigMSEs=[];
requiredCloudList=unique(cameraPair(:,1:2));
for i=1:length(requiredCloudList)
    [srcDesp{requiredCloudList(i)},srcSeed{requiredCloudList(i)},srcNorm{requiredCloudList(i)}] = extractEig(clouds{i},gridStep);
end
pairNum=size(cameraPair,1);
for i=1:pairNum
    m=cameraPair(i,1);
    d=cameraPair(i,2);
    Model=clouds{m}.Location(1:res:end,:)';
    Data= clouds{d}.Location(1:res:end,:)';
    T = eigMatch(srcDesp{m},srcDesp{d},srcSeed{m},srcSeed{d},srcNorm{m},srcNorm{d},overlap,gridStep);
    T = inv(T); %Ϊ���������Ч�ʣ����Ի�������
    R0= T(1:3,1:3);
    t0= T(1:3,4);
    [MSE,R,t] = TrICP(Model, Data, R0, t0, 100, overlap); %��������ر��ʱ��
    %     num= num+1
    if (MSE > 2.0*mean(eigMSEs))
        continue;
    end
    eigMSEs= [eigMSEs, MSE];
    accMotion= Rt2M(R,t);
    motionInfo{i,1}=accMotion;
    motionInfo{i,2}=m;
    motionInfo{i,3}=d;
end
end