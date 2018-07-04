% function [R, t, cMSE, CurrStep, corr] = ICP(R, t, MoveStep)
function [R, t] = ICP(Mns, Model, Data, R, t, MoveStep)
% global Model Data 

CurrStep = 0;
% Mns= createns(Model');
TData = transform_to_global(Data, R, t);
pMSE= 10^2;
cMSE= 10;
while ((CurrStep < MoveStep)&(abs(pMSE-cMSE)>10^-6))
    [corr,TD] = knnsearch(Mns,TData(1:3,:)');   
    corr(:,2) = [1 : length(corr)]';  
    ii = find(TD > 1.5*mean(TD));
    corr(ii,:) = [];
    TD(ii)= [];
    pMSE= cMSE;
    [R, t] = reg(Model(1:3,:), Data(1:3,:), corr);  
    cMSE= sqrt(mean(TD.^2));
    TData = transform_to_global(Data(1:3,:), R, t);
    CurrStep= CurrStep+1;   
end


%-----------------------------------------------------------------
function [R1, t1] = reg(Model, Data, corr)
% global Model Data
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