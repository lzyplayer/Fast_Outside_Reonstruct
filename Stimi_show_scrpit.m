load hannover2_GrtM_z.mat
routeDisplay(MotionGlobalBackup,'b-*',true,[]);
% turnpoint=abs(turn)>turnThreshold;
% turnPoints=[];
% for tu=1:length(turnHead)
%     turnPoints=[turnPoints,turnHead(tu)-1:turnHead(tu)+turnLengthNum-2];
% end
routeDisplay(MotionGlobal,'r-o',false,[]);%443:443+3,462:462+3,491:491+3,495:495+3,526:526+3

routeDisplay(GrtM,'g-d',false,[]);%(1:182)(1:532)799,490

% obtainResult(clouds,MotionGlobal,true);





