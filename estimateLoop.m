function [ cameraPosePair ] = estimateLoop( globalCameraPosition,cameraPosePair,LoopDectNum )
%ESTIMATELOOP �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
    
    curr=length(globalCameraPosition);
    for i=1:curr-LoopDectNum
        distance=norm(globalCameraPosition(curr,1:2)-globalCameraPosition(i,1:2));
        if(distance)<0.1
            pairInfo=[i curr distance];
            cameraPosePair=[cameraPosePair;pairInfo];
           
        end
    end
end

