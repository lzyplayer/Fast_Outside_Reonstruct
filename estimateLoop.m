function [ cameraPosePair ,flag] = estimateLoop( globalCameraPosition,cameraPosePair,LoopDectNum ,maxDis,flag )
%ESTIMATELOOP �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
addNum=0;
curr=length(globalCameraPosition);
currPair=[];

for i=1:curr-LoopDectNum
    maxDis=(curr-i)*0.345-12.5;
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
    if(size(sortedPair,1)>3)
        cameraPosePair=[cameraPosePair;sortedPair(1:3,:)];
    else
        cameraPosePair=[cameraPosePair;sortedPair];
    end
    end
end

