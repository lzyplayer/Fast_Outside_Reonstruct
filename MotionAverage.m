function Motion= MotionAverage(updatedMotion,Motion,D,num,N,FirstLoopNum,loopNumList)

erro= 10;
Dpi= pinv(D);
ii= 0;
while (erro>10^(-4)) && (ii<100)
   V= [];    Vo= [];
   ii=ii+1;
    for id= 1:num
        belongLoop=find(updatedMotion{id,3}<=loopNumList,1);
        weight=1/belongLoop;%
        i= updatedMotion{id,2};
        j= updatedMotion{id,3};
        Mij= updatedMotion{id,1};
        dMij= Motion{i}*Mij*inv(Motion{j});
        dmij= logm(dMij);
        vij= matrix2vec(dmij);
        V= [V;vij*weight];
    end
    
    dV= Dpi*V;
    for i= FirstLoopNum:N
        id= i*6-5;
        Motion{i}= expm(vec2matrix(dV(id:id+5,:)))*Motion{i};
        Motion{i}=real(Motion{i});
        Vo= [Vo;dV(id:id+2)];
    end
    erro= sum(abs(Vo));
end