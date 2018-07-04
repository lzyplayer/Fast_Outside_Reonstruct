function  routeDisplay( MotionGlobal ,type ,newFigure)
%ROUTEDISPLAY 此处显示有关此函数的摘要
%   此处显示详细说明
if(newFigure)
    figure;
end
route=[];
for p=1:length(MotionGlobal)
    route=[route; MotionGlobal{p}(1:3,4)'];
end
plot(route(:,1),route(:,2),type);
xlabel('x');
ylabel('y');
hold on;

% route=[];
% for p=1:length(MotionGlobal)
%     route=[route; MotionGlobal{p}(1:3,4)'];
% end
% plot3(route(:,1),route(:,2),route(:,3),'-*');
% xlabel('x');
% ylabel('y');
% zlabel('z');
% hold on;
end

