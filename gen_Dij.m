function  D  = gen_Dij(motionPair,N,loopNumList)
pnum=size(motionPair,1);
D=[];
for i=1:pnum
%     if isempty( motionPair{i,1})
%         continue;
%     end
    belongLoop=find(motionPair{i,3}<=loopNumList,1);
    weight=1/belongLoop;%
    Dij= zeros(6,6*N);
    id1= motionPair{i,2}*6-5;
    id2= motionPair{i,3}*6-5;
    Dij(1:6,(id1:(id1+5)))= -eye(6)*weight;
    Dij(1:6,(id2:(id2+5)))= eye(6)*weight;
    D=[D;Dij];
end