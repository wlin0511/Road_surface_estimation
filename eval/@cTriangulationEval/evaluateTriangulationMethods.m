function evaluateTriangulationMethods(gtTveCurrent, in_P_N1, in_P_N2, intrinsics)

% global triangErrorHistoMidPoint triangErrorHistoFast triangErrorHistoSmd
% 
% % Create histograms on the first call
% if isempty(triangErrorHistoSmd)
%     triangErrorHistoMidPoint = CHistogram(80, 4, 0.0, 50.0, -1.0); % histogram to collect triangulation errors over time
%     triangErrorHistoFast = CHistogram(80, 4, 0.0, 50.0, -1.0); % histogram to collect triangulation errors over time
%     triangErrorHistoSmd = CHistogram(80, 4, 0.0, 50.0, -1.0); % histogram to collect triangulation errors over time
% end

% parameters used for simulation of optical flow
simParams.flow.std_I = [0.0;0.0]; % [0;0]; %[0.7;0.7]; % noise added to end-correspondence [pix]
simParams.flow.minPointDist = 3; % min. point distance to be accounted for in simulation [m]
simParams.flow.minParallaxDeg = 0.0; % min. parallax angle (triangluation) to be accounted for in simulation [deg]
simParams.flow.simulatedPointsAtInfinity = false;
simParams.flow.infDist = 150;
simParams.flow.frameDtSec = 0.066;
simParams.flow.addWsDist = false;
simParams.flow.addLensDist = false;
simParams.flow.rollingShutterType = 'none';

flowSimulatorTriang = cFlowSimulator(intrinsics, simParams.flow);



% % % % Get parallax angles and sort input correspondences according to it
% % % parallaxAngsRad = angRadBetw2Rays(in_P_N1, in_P_N2);
% % % [parallaxAngsRadSorted, sortIdx] = sort(parallaxAngsRad);
% % % in_P_N1 = in_P_N1(:,sortIdx);
% % % in_P_N2 = in_P_N2(:,sortIdx);
% % % 
% % % idsValidParallax = find(rad2deg(parallaxAngsRadSorted) > simParams.flow.minParallaxDeg);
% % % in_P_N1 = in_P_N1(:,idsValidParallax);
% % % in_P_N2 = in_P_N2(:,idsValidParallax);
% % % parallaxAngsRadSorted = parallaxAngsRadSorted(idsValidParallax);
% % % 
% % % %parallaxAngsRad2 = angRadBetw2Rays(vec2unitVec(gt_P_N1), vec2unitVec(gt_P_N2));
% % % %assert(numel(find(parallaxAngsRad<deg2rad(simParams.flow.minParallaxDeg)-0.0000001)) == 0, '!!!');
% % % 
% % % % Triangulate 3D points in C1 for given flow field and camera motion
% % % [gt_P_C1, triangulatedPointsIdx] = flowSimulatorTriang.triangulatePointsFromFlow(gtTveCurrent.R_C2C1, gtTveCurrent.t_C2C1_C2, in_P_N1, in_P_N2);
% % % 
% % % % Thin out point cloud used for simulation
% % % gt_P_C1 = gt_P_C1(:,1:2:size(gt_P_C1,2));


gen_P_C1 = genPointCloudForTriangulationTest(intrinsics);
gen_P_C2 = transformFromFrame_EUC_R(gen_P_C1, gtTveCurrent.R_C2C1, gtTveCurrent.t_C2C1_C2)
triangulatedPointsIdx = 1 : size(gen_P_C1,2);


fig3D = figure();
clf;
hold on;
plot3(gen_P_C1(1,:), gen_P_C1(2,:), gen_P_C1(3,:), '*b');
plotCos3(eye(3), [0;0;0], 1);
xlabel('x'), ylabel('y'), zlabel('z');
axis equal;

numPoints = size(gen_P_C1,2);

simTveCurrent = cTve();
simTveCurrent.copy(gtTveCurrent);
% simTveCurrent.tx_C2C1_N2 = simTveCurrent.tx_C2C1_N2 + 0.0005;

if (sum(simParams.flow.std_I)==0)
    numMonteCarloIter = 2; %0000;
else
    numMonteCarloIter = 10000;
end

%triangErrorMatMid = NaN(numMonteCarloIter, numPoints);
%triangErrorMatFast = NaN(numMonteCarloIter, numPoints);

%res_zC2_Mid = NaN(numMonteCarloIter, numPoints);
%res_zC2_Fast = NaN(numMonteCarloIter, numPoints);


res_PC2_Mid = NaN(3, numPoints, numMonteCarloIter);
res_PC2_Fast = NaN(3, numPoints, numMonteCarloIter);
res_PC2_Smd = NaN(3, numPoints, numMonteCarloIter);

parallaxAngsRad = NaN(1,numPoints);

% Get parallax angles
[sim_P_N1, sim_P_N2, gt_P_N1, gt_P_N2, gt_P_C2, gt_P_C1, validPointsIdx] = flowSimulatorTriang.getFlowOfPointCloud( ...
    gen_P_C1, gtTveCurrent.R_C2C1, gtTveCurrent.t_C2C1_C2);
simulatedPointsIdx = triangulatedPointsIdx(validPointsIdx);
%
parallaxAngsRad(validPointsIdx) = angRadBetw2Rays(gt_P_N1, gt_P_N2);


for mcIter = 1 : numMonteCarloIter
    
    % resimulation of flow field
    [sim_P_N1, sim_P_N2, gt_P_N1, gt_P_N2, gt_P_C2, gt_P_C1, validPointsIdx] = flowSimulatorTriang.getFlowOfPointCloud( ...
        gen_P_C1, gtTveCurrent.R_C2C1, gtTveCurrent.t_C2C1_C2);
    simulatedPointsIdx = triangulatedPointsIdx(validPointsIdx);

    % Triangulate 3D points for given flow field and camera motion
    %
    % Mid-point method
    triangMid_P_C2 = triangulatePoints_MidMy(gtTveCurrent.R_C2C1, gtTveCurrent.t_C2C1_C2, sim_P_N1, sim_P_N2);  
    %triangMid_P_C1 = triangulatePointsRel_MID(gtTveCurrent.R_C2C1, gtTveCurrent.t_C2C1_C2, sim_P_N2, sim_P_N1);
    %triangMid_P_C2 = transformFromFrame_EUC_R(triangMid_P_C1, gtTveCurrent.R_C2C1, gtTveCurrent.t_C2C1_C2);
    
    % Find any results < 1m and set them to NaN
    idxBelowOneM = find(abs(triangMid_P_C2(3,:)) < 1);
    triangMid_P_C2(:,idxBelowOneM) = NaN;

    res_PC2_Mid(:,simulatedPointsIdx,mcIter) = triangMid_P_C2;
    
    

    % Fast (my)
    [triangFast_P_C2] = triangluateVirtHyperplane(sim_P_N1, sim_P_N2, simTveCurrent.e_C2C1, simTveCurrent.R_C1C2, simTveCurrent.t_C1C2_C1, []);
    
    % Find any results < 1m and set them to NaN
    idxBelowOneM = find(abs(triangFast_P_C2(3,:)) < 1);
    triangFast_P_C2(:,idxBelowOneM) = NaN;
    
    res_PC2_Fast(:,simulatedPointsIdx,mcIter) = triangFast_P_C2;
    
    % SMD
    [triangSmd_P_C2] = triangluateSmd(sim_P_N1, sim_P_N2, simTveCurrent.R_C2C1, simTveCurrent.t_C2C1_C2);
    
    % Find any results < 1m and set them to NaN
    idxBelowOneM = find(abs(triangSmd_P_C2(3,:)) < 1);
    triangSmd_P_C2(:,idxBelowOneM) = NaN;
    
    res_PC2_Smd(:,simulatedPointsIdx,mcIter) = triangSmd_P_C2;
    
    
    

    
%     if (numel(find(triangMid_P_C2(3,:)<0)) > 0)
%        t = 0;
%        find(~isfinite(triangMid_P_C1(3,:)))
%        find(~isfinite(triangFast_P_C2(3,:)))
%        find(~isfinite(triangSmd_P_C2(3,:)))
%        
%        triangMid_P_C1(:,77)
%        triangFast_P_C2(:,77)
%        triangSmd_P_C2(:,77)
%        
%        test = triangluateSmd(sim_P_N1(:,77), sim_P_N2(:,77), simTveCurrent.R_C2C1, simTveCurrent.t_C2C1_C2)
%        triangMid_P_C1 = triangulatePointsRel_MID(gtTveCurrent.R_C2C1, gtTveCurrent.t_C2C1_C2, sim_P_N2(:,57), sim_P_N1(:,57));
%     end
    
end

% Triangluation results per point (rows for different triangluation methods)
ResDepthMean_pC2 = NaN(3, numPoints);
ResDepthStd_pC2 = NaN(3, numPoints);
ResInvDepthMean_pC2 = NaN(3, numPoints);
ResInvDepthStd_pC2 = NaN(3, numPoints);
ResDepthErr_pC2 = NaN(3, numPoints);
ResErrMean_pC2 = NaN(3, numPoints);
ResAbsErrMeanDepht_pC2 = NaN(3, numPoints);

ResAgsErrMeanZfromIZ =  NaN(3, numPoints);
ResAgsErrMeanZ =  NaN(3, numPoints);
ResAgsErrMeanIZ =  NaN(3, numPoints);


triangMethods = {'Mid', 'Fast', 'Smd'};

figHandles(1) = figure(6669)
clf, hold on, xlabel('x/|xyz| [1/m]'), ylabel('y/|xyz| [1/m]'), zlabel('z/|xyz| [1/m]');
title('Triangulation Result Mid-Point (1000 MC-Iterations)');
%axis([-1 8 -25 250 -50 250]);
%set(gca,'Zdir','reverse');
%axis equal;
%plotCos3(eye(3), [0;0;0], 4);
figHandles(2) = figure(6670)
clf, hold on, xlabel('x/|xyz| [1/m]'), ylabel('y/|xyz| [1/m]'), zlabel('z/|xyz| [1/m]');
title('Triangulation Result Fast (1000 MC-Iterations)');
%axis([-1 8 -25 250 -50 250]);
%set(gca,'Zdir','reverse');
%axis equal;
%plotCos3(eye(3), [0;0;0], 4);
figHandles(3) = figure(6671)
clf, hold on, xlabel('x/|xyz| [1/m]'), ylabel('y/|xyz| [1/m]'), zlabel('z/|xyz| [1/m]');
title('Triangulation Result SMD (1000 MC-Iterations)');
%axis([-1 8 -25 250 -50 250]);
%set(gca,'Zdir','reverse');
%axis equal;
%plotCos3(eye(3), [0;0;0], 4);

for methodIdx = 1 : numel(triangMethods)
    
    % process one specific method
    figure(figHandles(methodIdx));
    clf;
    res_PC2_Method = eval(['res_PC2_', triangMethods{methodIdx}]);
    
    for pointIdx = 1 : 1 : 7 %numPoints
        
        % process one specific simulated point
        gen_p_C2 = gen_P_C2(:,pointIdx);
        res_pC2 = reshape(res_PC2_Method(:,pointIdx,:), 3, numMonteCarloIter);
        res_pC2 = res_pC2(:,isfinite(res_pC2(1,:))); % remove nan results
%         if any(isnan(res_pC2))
%             continue;
%         end
        if size(res_pC2,2) < 2
            continue;
        end
        

        
        
    
        resDepth_pC2 = sqrt( sum(res_pC2.^2, 1) );      % distances of triangluated points
        resInvDepth_pC2 = 1 ./ resDepth_pC2;            % inverse distances of triangluated points
        resInvDepthMean_pC2 = mean(resInvDepth_pC2);    % mean of inverse distances of triangluated points
        resDepthMean_pC2 = 1 / resInvDepthMean_pC2;     % mean of distances of triangluated points (transformed back from inverse depth)
        genDepth_p_C2 = norm(gen_p_C2);                 % distance of generated/simulated point (ground truth)
        genInvDepth_p_C2 = 1 / genDepth_p_C2;           % inverse distance of generated/simulated point (ground truth)
        resAbsErrMeanDepht_pC2 = abs(resDepthMean_pC2 - genDepth_p_C2);         % error in depth space
        ResAbsErrMeanDepht_pC2(methodIdx, pointIdx) = resAbsErrMeanDepht_pC2;   % 
        
        resZ = res_pC2(3,:);
        resInvZ = 1 ./ resZ;
        %meanResZ = mean(resZ);
        meanInvResZ = mean(resInvZ);
        meanResZ = mean(resZ);
        meanResZfromIZ = 1 / meanInvResZ;
        gtZ = gen_p_C2(3);
        gtIZ = 1 / gtZ;
        ResAgsErrMeanZ(methodIdx, pointIdx) = meanResZ - gtZ;
        ResAgsErrMeanZfromIZ(methodIdx, pointIdx) = meanResZfromIZ - gtZ;
        ResAgsErrMeanIZ(methodIdx, pointIdx) = meanInvResZ - gtIZ;
        
        if (pointIdx == 1) || (pointIdx == 5)
            figure()
            subplot(2,1,1);
            hold on;
            title(['IZ: Point ' num2str(pointIdx) ' Method ' num2str(methodIdx)]);
            plot(resInvZ, zeros(size(resInvZ)), '.b');
            %plotUncertainEllip3D(resCov_pC2inv, resMean_pC2inv, 3.841, 'm', 0.1);
            plot(meanInvResZ, 0, '*r');
            plot(gtIZ, 0, '*g');
            subplot(2,1,2);
            hold on;
            title(['Z: Point ' num2str(pointIdx) ' Method ' num2str(methodIdx)]);
            plot(resZ, zeros(size(resZ)), '.b');
            %plotUncertainEllip3D(resCov_pC2inv, resMean_pC2inv, 3.841, 'm', 0.1);
            plot(meanResZ, 0, '*r');
            plot(meanResZfromIZ, 0, '*m');
            plot(gtZ, 0, '*g');
        end
        
        depthHistoBins = -100 : 1.5 : 100;
        invDepthHistoBins = -1 : 0.01 : 1;
        
%         subplot(2,1,1);
%         hold on;
%         %plot(resInvDepth_pC2);
%         histogram(resInvZ, invDepthHistoBins);
%         plot(gtInvZ, 0, '*');
%         plot(meanInvResZ, 0, 'o');
%         
%         subplot(2,1,2);
%         hold on;
%         histogram(resZ, depthHistoBins);
%         plot(gtZ, 0, '*');
%         plot(meanResZ, 0, 'o');
%         
%         
% 
%         
%         subplot(2,1,1);
%         hold on;
%         %plot(resInvDepth_pC2);
%         histogram(resInvDepth_pC2, invDepthHistoBins);
%         plot(genInvDepth_p_C2);
%         plot(resInvDepthMean_pC2);
%         
%         subplot(2,1,2);
%         hold on;
%         histogram(resDepth_pC2, depthHistoBins);
%         plot(genDepth_p_C2);
%         plot(resDepthMean_pC2);
        
        
        res_pC2inv = res_pC2 ./ repmat(resDepth_pC2, 3, 1);
        resCov_pC2inv = cov(res_pC2inv'); % this has no meaning
        % determine mean in inverse space (because here we are close to normal distr.)
        resMean_pC2inv = mean(res_pC2inv')';
        % transform mean back to depth space
        resMean_pC2fromInv = norm(resMean_pC2inv) ./ resMean_pC2inv;
        % determine error
        resErrMean_pC2 = norm(gen_p_C2 - resMean_pC2fromInv);
        ResErrMean_pC2(methodIdx, pointIdx) = resErrMean_pC2;
        
%         ResDepthStd_pC2(methodIdx, pointIdx) = std(resDepth_pC2);
%         ResDepthMean_pC2(methodIdx, pointIdx) = mean(resDepth_pC2);
%         ResDepthErr_pC2(methodIdx, pointIdx) = abs(ResDepthMean_pC2(methodIdx, pointIdx)-norm(gen_p_C2));
% 
%         resInvDepth_pC2 = 1 ./ resDepth_pC2;
%         ResInvDepthStd_pC2(methodIdx, pointIdx) = std(resInvDepth_pC2);
%         ResInvDepthMean_pC2(methodIdx, pointIdx) = mean(resInvDepth_pC2);
        
        %res_pC2inv = [res_pC2(1:2,:); 1./res_pC2(3,:)];
        %res_pC2inv = res_pC2 ./ repmat(res_pC2(3,:), 3, 1);
        
        %genDepth_pC2 = sqrt( sum(gen_P_C2.^2, 1) );
        gen_p_C2inv = gen_p_C2 ./ norm(gen_p_C2);
        resMean_pC2inv = mean(res_pC2inv');
        
        resCov_pC2 = cov(res_pC2');
        resMean_pC2 = mean(res_pC2');

        if (0)
            figure()
            subplot(2,1,1);
            hold on;
            title(['Inverse depth: Point ' num2str(pointIdx) ' Method ' num2str(methodIdx)]);
            plot3(res_pC2inv(1,:), res_pC2inv(2,:), res_pC2inv(3,:), '.b');
            plotUncertainEllip3D(resCov_pC2inv, resMean_pC2inv, 3.841, 'm', 0.1);
            plot3(resMean_pC2inv(1), resMean_pC2inv(2), resMean_pC2inv(3), '*r');
            plot3(gen_p_C2inv(1), gen_p_C2inv(2), gen_p_C2inv(3), '*g');
            
            subplot(2,1,2);
            hold on;
            title(['Depth: Point ' num2str(pointIdx) ' Method ' num2str(methodIdx)]);
            hold on;
            plot3(res_pC2(1,:), res_pC2(2,:), res_pC2(3,:), '.b');
            plotUncertainEllip3D(resCov_pC2, resMean_pC2, 3.841, 'b', 0.05);
            plot3(resMean_pC2(1), resMean_pC2(2), resMean_pC2(3), '*r');
            plot3(resMean_pC2fromInv(1), resMean_pC2fromInv(2), resMean_pC2fromInv(3), '*m');
            plot3(gen_p_C2(1), gen_p_C2(2), gen_p_C2(3), '*g');
            
        end
        
        %plot3(gt_P_C1(1,:), gt_P_C1(2,:), gt_P_C1(3,:), '.r');
        %plot3(g_P_C2(1,pointIdx), gt_P_C2(2,pointIdx), gt_P_C2(3,pointIdx), '*g');
        
        
        
        
        %covtest = resCov_pC2
        %restest = res_pC2
        %isnan(res_pC2)
       
    end
end

%axis equal
%axis vis3d

ResAgsErrMeanZ
ResAgsErrMeanZfromIZ
ResAgsErrMeanIZ

depthHistoBins = 0 : 0.8 : 100;

figure(6667)
clf, hold on;
%subplot(2,1,1), hold on;
invDepthHistoBins = flip(1./depthHistoBins); %min(ResInvDepthStd_pC2(:)) : (max(ResInvDepthStd_pC2(:)) - min(ResInvDepthStd_pC2(:))) / 50 : max(ResInvDepthStd_pC2(:));
[parallaxAngsSorted, sortedIdx] = sort(parallaxAngsRad);
plot(rad2deg(parallaxAngsSorted), ResAbsErrMeanDepht_pC2(1,sortedIdx));
plot(rad2deg(parallaxAngsSorted), ResAbsErrMeanDepht_pC2(2,sortedIdx));
plot(rad2deg(parallaxAngsSorted), ResAbsErrMeanDepht_pC2(3,sortedIdx));
xlabel('parallax angle [deg]'), ylabel('mean error triang. depth [m]');
legend('Mid', 'Fast', 'Smd');
% subplot(2,1,2), hold on;
% plot(rad2deg(parallaxAngsSorted), ResDepthErr_pC2(1,sortedIdx));
% plot(rad2deg(parallaxAngsSorted), ResDepthErr_pC2(2,sortedIdx));
% plot(rad2deg(parallaxAngsSorted), ResDepthErr_pC2(3,sortedIdx));
% xlabel('parallax angle [deg]'), ylabel('err. triang. depth [m]');
% legend('Mid', 'Fast', 'Smd');



%legend('Mid', 'Smd');
title('Std. of Inverse Depth Estimates');

figure(6668)
clf, hold on;
invDepthHistoBins = flip(1./depthHistoBins); %min(ResInvDepthStd_pC2(:)) : (max(ResInvDepthStd_pC2(:)) - min(ResInvDepthStd_pC2(:))) / 50 : max(ResInvDepthStd_pC2(:));
[parallaxAngsSorted, sortedIdx] = sort(parallaxAngsRad);
plot(rad2deg(parallaxAngsSorted), ResInvDepthStd_pC2(1,sortedIdx));
plot(rad2deg(parallaxAngsSorted), ResInvDepthStd_pC2(2,sortedIdx));
plot(rad2deg(parallaxAngsSorted), ResInvDepthStd_pC2(3,sortedIdx));
% [Nmid, edges] = histcounts(ResInvDepthStd_pC2(1,:), invDepthHistoBins);
% [Nfast, edges] = histcounts(ResInvDepthStd_pC2(2,:), invDepthHistoBins);
% [Nsmd, edges] = histcounts(ResInvDepthStd_pC2(3,:), invDepthHistoBins);
%bar(edges(1:end-1), [Nmid', Nfast', Nsmd'],'grouped')
%semilogx(edges(1:end-1), Nmid'); %, Nfast', Nsmd']
%semilogx(edges(1:end-1), Nfast'); %, Nfast', Nsmd']
%semilogx(edges(1:end-1), Nsmd'); %, Nfast', Nsmd']
%xlabel('std triang. inv. depth [m]'), ylabel('# samples');
xlabel('parallax angle [deg]'), ylabel('\sigma triang. inv. depth [m]');
legend('Mid', 'Fast', 'Smd');
%legend('Mid', 'Smd');
title('Std. of Inverse Depth Estimates');

figure(6667)
clf, hold on;

%histogram(ResDepthStd_pC2(1,:), depthHistoBins);
%histogram(ResDepthStd_pC2(2,:), depthHistoBins);
%histogram(ResDepthStd_pC2(3,:), depthHistoBins);
[Nmid, edges] = histcounts(ResDepthStd_pC2(1,:), depthHistoBins);
[Nfast, edges] = histcounts(ResDepthStd_pC2(2,:), depthHistoBins);
[Nsmd, edges] = histcounts(ResDepthStd_pC2(3,:), depthHistoBins);

bar(edges(1:end-1), [Nmid', Nfast', Nsmd'],'grouped')
xlabel('std triang. depth [m]'), ylabel('# samples');
legend('Mid', 'Fast', 'Smd');
title('Std. of Depth Estimates');

figure(6666)
clf, hold on;
plot(1:numel(ResInvDepthMean_pC2(1,:)), ResInvDepthMean_pC2(1,:));
plot(1:numel(ResInvDepthMean_pC2(1,:)), ResInvDepthMean_pC2(2,:));
plot(1:numel(ResInvDepthMean_pC2(1,:)), ResInvDepthMean_pC2(3,:));
%bar(edges(1:end-1), [Nmid', Nfast', Nsmd'],'grouped')
xlabel('std triang. depth [m]'), ylabel('# samples');
legend('Mid', 'Fast', 'Smd');
title('Mean. of Inverse Depth Estimates');

histogram(ResInvDepthStd_pC2(3,:), 20);

% resMean_zC2_Mid = mean(res_zC2_Mid,1);
% resStd_zC2_Mid = std(res_zC2_Mid,1);
% 
% figure(6667)
% histogram(meanTriangErrorMid)
% 
% figure(6668)
% histogram(stdTriangErrorMid)
% 
% 
% 
% 
% 
% 
% 
% figure(6670)
% clf, hold on;
% plot(1:numMonteCarloIter, res_zC2_Mid(:,12));
% plot(1:numMonteCarloIter, ones(numMonteCarloIter)*resMean_zC2_Mid(12));
% plotStd(1:numMonteCarloIter, ones(numMonteCarloIter)*resMean_zC2_Mid(12), ones(numMonteCarloIter)*resStd_zC2_Mid(12), 'b', 0.5);
% 
% figure(1345);
% clf;
% hold on;
% plot(rad2deg(parallaxAngsRadSorted), meanTriangErrorMid);
% plot(rad2deg(parallaxAngsRadSorted), meanTriangErrorFast);
% plotStd(rad2deg(parallaxAngsRadSorted), meanTriangErrorMid, stdTriangErrorMid, 'b', 0.5);
% xlabel('parallax [deg]'), ylabel('triang. error. [m]');
% 
% % Plot
% 
% [triangErrorFastSorted, sortedIdx] = sort(triangErrorFast);
% [pAngSorted, sortedIdx] = sort(parallaxAngRad);
% 
% figure(114);
% clf;
% hold on;
% plot(1:numel(triangErrorMid), triangErrorMid(sortedIdx));
% plot(1:numel(triangErrorFast), triangErrorFast(sortedIdx));
% 
% plotErrorHist(figure(111), triangErrorHistoMidPoint, 'Trianglulation Error Mid-Point');
% plotErrorHist(figure(112), triangErrorHistoFast, 'Trianglulation Error Fast');
% %plotErrorHist(figure(113), errorHistFromTriangResult, 'Trianglulation Error Mid-Point');

end

%%

function triangError = errorHistFromTriangResult(tr_P_C2, parallaxAngRad, gt_P_C2, simParams)

parallaxAngDeg = rad2deg(parallaxAngRad);
errorVecs = gt_P_C2 - tr_P_C2;
triangError = sqrt( sum(errorVecs.^2, 1) );
%histo.addSig(triangError, zeros(1,numel(triangError)));
%histo.update();

[N, x, y] = histcounts2(triangError, parallaxAngDeg, 0 : 0.5 : 30, simParams.flow.minParallaxDeg : 0.4 : 10);
clf;
surf(repmat(x(1:end-1),numel(y)-1,1), repmat(y(1:end-1)',1,numel(x)-1), N')
xlabel('triang. error. [m]'), ylabel('parallax [deg]'), zlabel('# samples');

end

function plotErrorHist(figHandle, histo, plotTitle)

figureNoFocus(figHandle);
clf;
axisH = subplot(1,1,1); %get(figHandles.triangulationErrors,'CurrentAxes'); %axis;
hold on;
histo.plotBins(axisH);
%triangErrorHisto.plotConvBins(axisH);
histo.plotOuterBins(axisH);
histo.plotPeak(axisH);
xlim([histo.minVal, histo.maxVal + histo.binRange]);
xlabel('triangulation error [m]');
ylabel('# samples');
title(plotTitle);

end



