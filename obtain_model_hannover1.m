function allPointCloud=obtain_model_hannover1(clouds, motion)
Model = [];
for i=1:length(clouds)
    currMotion=motion{i};
    Location=clouds{i}.Location';
    TLocation=currMotion*[Location;ones(1,size(Location,2))];
    Tdata = TLocation(1:3,:);
    Model = [Model, Tdata];
end
allPointCloud=pointCloud(Model');
reAxes = pcshow(allPointCloud);
reAxes.CameraPosition=[-3037.32 -4448.29 -5802.49];
reAxes.CameraTarget=[-154.458 346.5108 -457.597];
reAxes.CameraUpVector=[0.35594 0.59201 -0.72306];
reAxes.CameraViewAngle=14.01893;