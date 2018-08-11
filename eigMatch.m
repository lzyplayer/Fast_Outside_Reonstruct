function T = eigMatch(srcDesp,tarDesp,srcSeed,tarSeed,srcNorm,tarNorm,overlap,gridStep)
%% parameter configuration for flann search
params.algorithm = 'kdtree';
params.trees = 8;
params.checks = 64;
radii = (0.5:0.5:2)*gridStep;

[srcIdx,dist] = flann_search(srcDesp,tarDesp,1,params); % match with descriptors
[dist,id]= sort(dist);
%% aggregating each pair of correspondence for finding the best match
M = size(srcSeed,2);
N = size(tarSeed,2);
seedIdx = srcIdx; 
Err = inf(N,1);
tform = cell(1,N); 
ovNum = ceil(overlap*N); 
distThr = 0.2/4*length(radii); 
thetaThr = 10; 
threshold = gridStep*gridStep;
  for i = 1:ceil(0.3*N)
    n= id(i);
%   for n = 1:N
    seed = srcSeed(:,seedIdx(n));
    seedNorm = srcNorm(:,seedIdx(n));
     %%  当前点特征向量与所有其他特殊点的內积
    % source point cloud
    d = bsxfun(@minus,srcSeed,seed);
    d = sqrt(sum(d.^2,1)); % distance
    inProd = bsxfun(@times,srcNorm,seedNorm);
    %分别取四个特征向量的首元素，中间元素，末尾元素
    %然后将每个特征向量各自sum，一共4个
    inProd = inProd(1:3:end,:) + inProd(2:3:end,:) + inProd(3:3:end,:);
    theta = real(acosd(inProd));  % inner product

    % target point cloud
    r = bsxfun(@minus,tarSeed,tarSeed(:,n));
    r = sqrt(sum(r.^2,1)); % distance
    inProd = bsxfun(@times,tarNorm,tarNorm(:,n));
    inProd = inProd(1:3:end,:) + inProd(2:3:end,:) + inProd(3:3:end,:);
    alpha = real(acosd(inProd));  % inner product   

    IDX = rangesearch(r',d',gridStep/2,'distance','cityblock');
    
    matches = [seedIdx(n) n];
    for m = [1:seedIdx(n)-1 seedIdx(n)+1:M]        
        idx = IDX{m};%find(abs(r-d(m))<gridStep/2);%
        if(isempty(idx))
            continue;
        end
        dTheta = bsxfun(@minus,alpha(:,idx),theta(:,m));
        dTheta = abs(dTheta);
        Tab = dTheta<thetaThr;
        Tab = sum(Tab,1);
        if(all(Tab<size(theta,1)))
            continue;
        end
        sim = mean(dTheta,1);
        sim(Tab<size(theta,1)) = inf;
        [minSim,ol] = min(sim);
        R = norm(srcDesp(:,m)-tarDesp(:,idx(ol)));
        if(minSim<thetaThr && R<distThr)
            matches = [matches; m idx(ol)];
        end
    end
    if(size(matches,1)>10)
        match_srcSeed = srcSeed(:,matches(:,1));
        match_tarSeed = tarSeed(:,matches(:,2));
        CS = ransac(double(match_srcSeed),double(match_tarSeed),threshold);   
        
        if(sum(CS)<3)
            continue;
        end
        
        match_srcSeed = match_srcSeed(:,CS);
        match_tarSeed = match_tarSeed(:,CS);
        [T, Eps] = estimateRigidTransform(match_tarSeed, match_srcSeed);
        tarEst = T*[srcSeed;ones(1,M)];
        tarEst = tarEst(1:3,:);
        tform{n} = T;
        
        [index,dist] = flann_search(tarEst,tarSeed,1,params);
        [dist,ind] = sort(dist);        
        Err(n) = sum(sum((tarEst(:,index(ind(1:ovNum)))-tarSeed(:,ind(1:ovNum))).^2));
    end
    if (size(matches,1)> 0.65*size(srcDesp,2))
        break;
    end
 end
[v,idx] = min(Err);
T = tform{idx};
% if(isempty(T))
%     disp(['match Failed with tarseed:' num2str(length(tarSeed)) ' srcSeed:' num2str(length(srcSeed)) ]);
% end
end
