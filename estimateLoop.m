function [ cameraPosePair ,flag] = estimateLoop( globalCameraPosition,cameraPosePair,LoopDectNum ,flag,lastLoop,loopDisCal )
%ESTIMATELOOP 此处显示有关此函数的摘要
%   此处显示详细说明
curr=length(globalCameraPosition);
currPair=[];

for i=1:curr-LoopDectNum
%     maxDis=0.345*(curr-i)-19.4;         %sqrt(curr-lastLoopNum)*23;
    maxDis=loopDisCal(curr-i,curr-lastLoop);
    if isnan(maxDis) %|| (curr<680 && curr>630
        maxDis=30;
    end
    if (curr<680 && curr>630) %|| (curr<680 && curr>630
        maxDis=50;
    end 
    distance=norm(globalCameraPosition(curr,1:2)-globalCameraPosition(i,1:2));
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
    if(size(sortedPair,1)>4)
        cameraPosePair=[cameraPosePair;sortedPair(1:4,:)];
    else
        cameraPosePair=[cameraPosePair;sortedPair];
    end
    end
end

