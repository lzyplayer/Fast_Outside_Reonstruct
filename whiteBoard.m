for i=1:size(odometry,1)
    t=odometry(i,1:3)';
%     t(1)=t(1)-721787150;
    R=quat2rotm(odometry(i,4:7));
    Motion1{i}=(Rt2M(R,t));  %ÐèÒª¿¼Á¿
    firstCentralMotion{i}=(Motion1{1})\Motion1{i};
end
routeDisplay(Motion1,'-*',true,[]);
