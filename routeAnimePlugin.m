function lineHandle = routeAnimePlugin(MotionGlobal,type ,axe )
%ROUTEANIMEPLUGIN 此处显示有关此函数的摘要
%   此处显示详细说明
route=[];
if(iscell(MotionGlobal))
    for p=1:length(MotionGlobal)
        route=[route; MotionGlobal{p}(1:3,4)'];
    end
else
    route=MotionGlobal;
end
hold on
axis manual

axis equal
lineHandle=plot3(axe,route(:,1),route(:,2),route(:,3),type);
axis(axe,[-4 1 -1 3 -0.5 0.5]);
% drawnow()

end

