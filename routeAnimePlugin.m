function lineHandle = routeAnimePlugin(MotionGlobal,type ,axe )
%ROUTEANIMEPLUGIN �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
route=[];
if(iscell(MotionGlobal))
    for p=1:length(MotionGlobal)
        route=[route; MotionGlobal{p}(1:3,4)'];
    end
else
    route=MotionGlobal;
end

lineHandle=plot(axe,route(:,1),route(:,2),type);
axis equal
end

