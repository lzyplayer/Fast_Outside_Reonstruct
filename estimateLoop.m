function [ cameraPosePair ,flag] = estimateLoop( globalCameraPosition,cameraPosePair,LoopDectNum ,flag,maxDis )
%ESTIMATELOOP �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
addNum=0;
curr=length(globalCameraPosition);
currPair=[];
if length(globalCameraPosition)==794
    0==0;
end

for i=1:curr-LoopDectNum
%     maxDis=0.345*(curr-i)-19.4;         %sqrt(curr-lastLoopNum)*23;
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

