p=zeros(length(clouds),1);
for i=1:length(clouds)
    volume=abs((clouds{i}.XLimits(2)-clouds{i}.XLimits(1))*(clouds{i}.YLimits(2)-clouds{i}.YLimits(1))*(clouds{i}.ZLimits(2)-clouds{i}.ZLimits(1)));
    p(i)=clouds{i}.Count/volume;
    
end
aveP=mean(p);
normalP=1.2980e+07;
normalG=0.01;
value=nthroot(((normalP*(normalG^3))/(aveP)),3)
