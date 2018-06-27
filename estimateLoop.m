function [ cameraPosePair ,flag] = estimateLoop( globalCameraPosition,cameraPosePair,LoopDectNum ,flag )
%ESTIMATELOOP 此处显示有关此函数的摘要
%   此处显示详细说明

curr=length(globalCameraPosition);
for i=1:curr-LoopDectNum
    distance=norm(globalCameraPosition(curr,1:2)-globalCameraPosition(i,1:2));
    if(distance)<0.03
        pairInfo=[i curr distance];
        cameraPosePair=[cameraPosePair;pairInfo];
        if(flag == 0)
            flag=1;
        end
    end
end
end

