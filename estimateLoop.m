function [ cameraPosePair ,flag] = estimateLoop( globalCameraPosition,cameraPosePair,LoopDectNum ,flag,lastLoopNum ,maxDis)
%ESTIMATELOOP 此处显示有关此函数的摘要
%   此处显示详细说明
addNum=0;
keepCirForFrame=3;
curr=length(globalCameraPosition);
currPair=[];


for i=1:curr-LoopDectNum

    distance=norm(globalCameraPosition(curr,1:2)-globalCameraPosition(i,1:2));%     maxDis=0.345*(curr-i)-19.4;         %sqrt(curr-lastLoopNum)*23;
%     maxDis=loopDisCal(curr-i,curr-lastLoopNum); %lastLoopNum
%     if isnan(maxDis) || maxDis<5
%         maxDis=20;
%     end
    if(distance)< maxDis
        pairInfo=[i curr distance];
        currPair=[currPair;pairInfo];
        if(flag == 0)
            flag=1;
        end
    end
end
    if (~isempty(currPair))
    sortedPair=sortrows(currPair,3);
    if(size(sortedPair,1)>keepCirForFrame)
        cameraPosePair=[cameraPosePair;sortedPair(1:keepCirForFrame,:)];
    else
        cameraPosePair=[cameraPosePair;sortedPair];
    end
    end
end

