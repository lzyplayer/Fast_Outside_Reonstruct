clc;clear;
close all;

filepathWp='./data/hannover2/hannover2/scan3d_0_';
% dirInfo=dir('./data/dat_et4/');
scannum=924   ;%(length(dirInfo)-2)/2;
% lastnum=scannum-1;
route=[];
% clouds=cell(scannum,1);
for k=1:scannum
    ordernum=num2str(k);
%     while(length(ordernum)<3)
%         ordernum=['0' ordernum];
%     end
    prefix=[ordernum '.3d'];
    %读取.3d
    data_raw=importdata([filepathWp prefix]);
    objectbad=(data_raw(:,1).^2+data_raw(:,2).^2+data_raw(:,3).^2>850e+6);
    objectGround=(data_raw(:,3)>(-0.3*1e+3));
    objectSelect=~(objectbad |objectGround);
    
    clouds{k}=pointCloud(data_raw(objectSelect,1:3).*1e-2);
%     
%     clouds{k}=pointCloud(data_raw(:,1:3));
%     pcshow(clouds{k});
%     hold on
    %读取.pose  route only
%         prefix=[ordernum '.pose'];
%         scanpose{k+1}=load([filepathWp prefix]);
% %         R=OulerToRota(scanpose{k+1}(2,:));
%         T=scanpose{k+1}(1,:)';
% %         Grt{k+1}=[R,T;0 0 0 1];
%         route=[route ; T'];
end
save hannover2zoomed.mat clouds;




%     fidin = fopen([filepathWp prefix],'r');
%     nline = 0;
%     currscan=zeros(81360,4);
%     while ~feof(fidin)         %判断是否为文件末尾
%         nline = nline+1;
%         tline = fgetl(fidin);         %从文件读行
%         if(nline>1)
%         currscan(nline-1,:) = str2num(tline);
%         end
%     end
%       fclose(fidin);
%     scan{k+1}=currscan;

% pose =importdata('./data/hannover2/hannover2/odometry_0_sync_interpol.dat');

