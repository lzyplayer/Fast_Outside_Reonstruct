function [ M ] = TrimmedICP(ns, model,data,initialMotion,id,iterationThreshlod,TrMin,TrMax,lamda)
%global ns

PreMSE= 10^5;   CurMSE= 10^6;  step = 1;
scan=initialMotion*data;
while(step < iterationThreshlod)&(abs(CurMSE-PreMSE)>10^(-6))
    [corr,TD] = knnsearch(ns{id},scan(1:3,:)');
    SortTD2 = sortrows(TD.^2); % Sort the correspongding points
    minTDIndex = floor(TrMin*length(TD)); % Get minimum index of TD
    maxTDIndex = ceil(TrMax*length(TD)); % Get maxmum index of TD    
    TDIndex = [minTDIndex : maxTDIndex]';
    mTr = TDIndex./length(TD);
    mCumTD2 = cumsum(SortTD2);
    mMSE = mCumTD2(minTDIndex : maxTDIndex)./TDIndex;
    mPhi = ObjectiveFunction(mMSE, mTr);  
    PreMSE=CurMSE;
    [CurMSE, nIndex] = min(mPhi);    
    Trim = mTr(nIndex); % Update Tr for next step    
    corr(:,2) = [1 : length(corr)]';
    % Sort the corresponding points
    corrTD = [corr, TD];
    SortCorrTD = sortrows(corrTD, 3);
    [M, TCorr, scan] = CalRtPhi(model, data, SortCorrTD, Trim);
    step= step+1;
end


end

%%%%%%%%%%%%%%%%%%%%Integrated Function%%%%%%%%%%%%%%%%%%%%
%% Calculate R,t,Phi based on current overlap parameter
function [M,TCorr,TData] = CalRtPhi(Model, scan, SortCorrTD,Tr)

TrLength = floor(Tr*size(SortCorrTD,1)); % The number of corresponding points after trimming
TCorr = SortCorrTD(1:TrLength, 1:2);     % Trim the corresponding points according to overlap parameter Tr
% Register MData with TData
[M] = reg(Model(1:3,:), scan(1:3,:), TCorr);
% To obtain the transformation data
TData = M*scan;

end
%%%%%%%%%%%%%%% Calculate the registration matrix %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%% T(TData)->MData %%%%%%%%%%%%%%%%%%%%%%%%%
% SVD solution
function [M] = reg(Model, Data, corr)

n = length(corr); 
M = Model(:,corr(:,1)); 
mm = mean(M,2);
S = Data(:,corr(:,2));
ms = mean(S,2); 
Sshifted = [S(1,:)-ms(1); S(2,:)-ms(2); S(3,:)-ms(3)];
Mshifted = [M(1,:)-mm(1); M(2,:)-mm(2); M(3,:)-mm(3)];
K = Sshifted*Mshifted';
K = K/n;
[U A V] = svd(K);
R1 = V*U';
if det(R1)<0
    B = eye(3);
    B(3,3) = det(V*U');
    R1 = V*B*U';
end
t1 = mm - R1*ms;
M=[];
M(1:3,1:3)=R1;
M(1:3,4)=t1;
M(4,:)=[0,0,0,1];
end
