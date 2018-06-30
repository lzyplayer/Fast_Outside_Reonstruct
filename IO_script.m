clc;clear;close all;

filepathWp='./data/dat_et4/scan';
dirInfo=dir('./data/dat_et4/');
scannum=(length(dirInfo)-2)/2;
lastnum=scannum-1;
route=[];
scan=cell(scannum,1);
for k=0:lastnum
    ordernum=num2str(k);
    while(length(ordernum)<3)
        ordernum=['0' ordernum];
    end
    prefix=[ordernum '.3d'];
    %��ȡ.3d
    scan{k+1}=load([filepathWp prefix]);
     disp(['scan ' num2str(k) ' loaded']);

    %��ȡ.pose
        prefix=[ordernum '.pose'];
        scanpose{k+1}=load([filepathWp prefix]);
        R=OulerToRota(scanpose{k+1}(2,:));
        T=scanpose{k+1}(1,:)';
        Grt{k+1}=[R,T;0 0 0 1];
        route=[route ; T'];
end





%     fidin = fopen([filepathWp prefix],'r');
%     nline = 0;
%     currscan=zeros(81360,4);
%     while ~feof(fidin)         %�ж��Ƿ�Ϊ�ļ�ĩβ
%         nline = nline+1;
%         tline = fgetl(fidin);         %���ļ�����
%         if(nline>1)
%         currscan(nline-1,:) = str2num(tline);
%         end
%     end
%       fclose(fidin);
%     scan{k+1}=currscan;



