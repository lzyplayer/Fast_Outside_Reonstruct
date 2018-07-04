function [errR, errT] = err_comp(grtR, grtT, motion, scannum, MSEs)

errR = 0;
errT = 0;
for i=1:scannum
    errR = errR + norm(motion{i}(1:3,1:3)-grtR{i},'fro');
    errT = errT + norm(motion{i}(1:3,4)-grtT{i},2);
end
errR = errR/scannum;
errT = errT/scannum;
