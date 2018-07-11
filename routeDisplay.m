function  routeDisplay( MotionGlobal ,type ,newFigure,specialshow)
%ROUTEDISPLAY 此处显示有关此函数的摘要
%   此处显示详细说明
if(newFigure)
    figure;
end
% route=[];
% for p=1:length(MotionGlobal)
%     route=[route; MotionGlobal{p}(1:3,4)'];
% end
% plot(route(:,1),route(:,2),type);
% xlabel('x');
% ylabel('y');
% hold on;

route=[];
if(iscell(MotionGlobal))
    for p=1:length(MotionGlobal)
        route=[route; MotionGlobal{p}(1:3,4)'];
    end
else
    route=MotionGlobal;
end
plot3(route(:,1),route(:,2),route(:,3),type);%zeros(length(route),1)
xlabel('x');
ylabel('y');
zlabel('z');
hold on;
axis equal;

plot3(route(specialshow,1),route(specialshow,2),route(specialshow,3),'dk','MarkerSize',10);

end

