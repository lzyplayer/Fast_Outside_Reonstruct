function  D  = gen_Dij(motionPair,N)
pnum=size(motionPair,1);
D=[];
for i=1:pnum
%     if isempty( motionPair{i,1})
%         continue;
%     end
    Dij= zeros(6,6*N);
    id1= motionPair{i,2}*6-5;
    id2= motionPair{i,3}*6-5;
    Dij(1:6,(id1:(id1+5)))= -eye(6);
    Dij(1:6,(id2:(id2+5)))= eye(6);
    D=[D;Dij];
end