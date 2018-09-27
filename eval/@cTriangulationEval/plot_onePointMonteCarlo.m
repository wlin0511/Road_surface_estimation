%% function: plot_onePointMonteCarlo
% author: <Lin, Wei> (CC-DA/ESV1)
% date: <2017-04-29> 
%
% 
%
%
% INPUTS
% cTriangEvalObj_onepoint : the object of the class cTriangulationEval (specially for onepoint simulation)
% P_C1 : the one single point to be simulated
% camMotion : the structure which carries camera motion data
% iteration_num : num of simulation iterations
%                             
%
%
% OUTPUTS


function plot_onePointMonteCarlo(obj, frameIdx, pointIdx, funcHandle_name  )

% est_P_C2 = zeros(3,iteration_num);
% sim_P_N2 = zeros(3,iteration_num);
% 
% for i = 1:1:iteration_num
%     [sim_P_N1, sim_P_N2(:,i), gt_P_N1, gt_P_N2, gt_P_C2, gt_P_C1, validPointsIdx] = cTriangEvalObj_onepoint.flowSimObj.getFlowOfPointCloud(P_C1, camMotion.R_C2C1, camMotion.t_C2C1_C2);
%     
%     % cTriangEvalObj = cTriangulationEval(funcHandlesTriang, simIntrinsics, params);
%     gt_P_N2_C1 = camMotion.R_C2C1' * gt_P_N2;
%     ang_rad = angRadBetw2Rays(gt_P_N1, gt_P_N2_C1);
%     est_P_C2(:,i) = cTriangEvalObj_onepoint.funcHandlesTriang{1}(sim_P_N1, sim_P_N2(:,i), camMotion);
% end

numFuncHandles = length(funcHandle_name);
numIteration = obj.params.numMonteCarloIterations;

gt_P_C2_point = double(obj.raw_data.gt_P_C2{1,frameIdx}(:, pointIdx));

gt_P_N1_point = double(obj.raw_data.gt_P_N1{1,frameIdx}(:, pointIdx));
gt_P_N2_point = double(obj.raw_data.gt_P_N2{1,frameIdx}(:, pointIdx));
gt_P_N2_C1_point = obj.raw_data.camMotion_gt{1, frameIdx}.R_C2C1' * gt_P_N2_point;
ang_rad = angRadBetw2Rays(gt_P_N1_point, gt_P_N2_C1_point);


sim_P_N1_pointSet = zeros(3, numIteration);
for iterationIdx = 1:numIteration
    sim_P_N1_pointSet(:, iterationIdx) = obj.raw_data.sim_P_N1{iterationIdx, frameIdx}(:, pointIdx);
end

for funcHandleIdx = 1:numFuncHandles
    
    
    est_P_C2_pointSet = zeros(3, numIteration);
    
    for iterationIdx = 1:numIteration
        est_P_C2_pointSet(:, iterationIdx) = obj.triang_data_sim.est_P_C2{funcHandleIdx, iterationIdx, frameIdx}(:, pointIdx);
    end

    %% plot x, y, inv_z(z) with uncertain ellip in C2 frame
    % x, y, inv_z
    % est_P_C2_inv_z = [est_P_C2(1,:);est_P_C2(2,:);ones(size(est_P_C2(3,:)))./est_P_C2(3,:)];
    figure('name',['onePointMC ', funcHandle_name{funcHandleIdx}], 'NumberTitle', 'off')
    subplot(1,2,1);
    scatter3(est_P_C2_pointSet(1,:),est_P_C2_pointSet(2,:),est_P_C2_pointSet(3,:),'.','MarkerFaceAlpha',.6,'MarkerEdgeAlpha',.6 )
    hold on
    scatter3(gt_P_C2_point(1),gt_P_C2_point(2),gt_P_C2_point(3),'m','filled')
    text(gt_P_C2_point(1),gt_P_C2_point(2),gt_P_C2_point(3),'gt_P_C2','Color','m', 'Interpreter','none')
    hold on
    est_P_C2_mean = double(mean(est_P_C2_pointSet,2));
    est_P_C2_cov = cov(est_P_C2_pointSet');
    plotUncertainEllip3D( est_P_C2_cov , est_P_C2_mean, 9.348, 'r', 0.5 )
    scatter3(est_P_C2_mean(1),est_P_C2_mean(2),est_P_C2_mean(3),'g','filled')
    text(est_P_C2_mean(1),est_P_C2_mean(2),est_P_C2_mean(3),'est_mean','Color','g', 'Interpreter','none')
    
    scatter3(0,0,0,'k')
    text(0,0,0,'C1','Color','r', 'Interpreter','none')
    xlabel('x'), ylabel('y'), zlabel('z')
    %axis equal  
    title({['est_P_C2 X Y Z ', funcHandle_name{funcHandleIdx}],...
           ['pointIdx ', num2str(pointIdx), ' frame ', num2str(frameIdx)]},'Interpreter','none' )
       
    %% plot x,z with uncertain ellip
%     est_P_C2_xz = [est_P_C2_pointSet(1,:);est_P_C2_pointSet(3,:)];
%     subplot(1,3,2);
%     scatter(est_P_C2_xz(1,:),est_P_C2_xz(2,:),'.', 'MarkerFaceAlpha',.6,'MarkerEdgeAlpha',.6)
%     hold on
%     scatter(gt_P_C2_point(1),gt_P_C2_point(3),'m','filled')
%     text(gt_P_C2_point(1),gt_P_C2_point(3),'gt_P_C2','Color','m', 'Interpreter','none')
%     hold on
%     est_P_C2_xz_mean = double(mean(est_P_C2_xz,2));
%     est_P_C2_xz_cov = cov(est_P_C2_xz');
%     plotUncertainEllip2D([], est_P_C2_xz_cov, est_P_C2_xz_mean, 'r', 7.378)
%     scatter(est_P_C2_xz_mean(1),est_P_C2_xz_mean(2),'g','filled')
%     text(est_P_C2_xz_mean(1),est_P_C2_xz_mean(2),'est_mean','Color','g', 'Interpreter','none')
%     %axis equal
%     xlabel('x')
%     ylabel('z')
%     title({['est_P_C2 X Z ', funcHandle_name{funcHandleIdx}],...
%            ['pointIdx ', num2str(pointIdx), ' frame ', num2str(frameIdx)]},'Interpreter','none' )
    %%  in C1 frame
    subplot(1,2,2);
    scatter3(sim_P_N1_pointSet(1,:),sim_P_N1_pointSet(2,:), sim_P_N1_pointSet(3,:), '.')
    hold on
    est_P_C1 = zeros(3, numIteration);
    for iterationIdx = 1:numIteration
        camMotion = obj.raw_data.camMotion_sim{iterationIdx, frameIdx};
        est_P_C1(:, iterationIdx) = camMotion.R_C2C1' *  est_P_C2_pointSet(:, iterationIdx) + camMotion.t_C1C2_C1;
    end
    
    scatter3( est_P_C1(1,:), est_P_C1(2,:), est_P_C1(3,:),'k.','MarkerFaceAlpha',.6,'MarkerEdgeAlpha',.6 )
    opt.imagesize = [obj.flowSimObj.simIntrinsics.h, obj.flowSimObj.simIntrinsics.w];
    opt.theta = obj.flowSimObj.simIntrinsics.fov_u;
    opt.f = 1;
    draw3DCamera(opt,eye(3),[0;0;0],'r')  % camera C1
    text(0,0,0,'C1','Color','red')
    %   P_C = R_CW * P_W + tCW_C   set the camera frame 1 as the world frame
    %  P_C2 = R_C2C1 * P_C1 + tC2C1_C2
    draw3DCamera(opt,obj.raw_data.camMotion_gt{1, frameIdx}.R_C2C1, obj.raw_data.camMotion_gt{1, frameIdx}.t_C2C1_C2,'g')  % camera C2
    
    % the origin of camera 2   in camera frame 1  : P_C1 = R_C1C2 * P_C2 + t_C1C2_C1   from frame C1 to frame C2
    C2_origin = double(obj.raw_data.camMotion_gt{1, frameIdx}.t_C1C2_C1);
    text(C2_origin(1),C2_origin(2),C2_origin(3),'C2','Color','red')
    
    axis square
    
    title({['est_P_C1 in C1 frame ', funcHandle_name{funcHandleIdx}],...
           ['pointIdx ', num2str(pointIdx), ' frame ', num2str(frameIdx)]},'Interpreter','none' )
    
    gt_P_C2_norm_signed = norm(gt_P_C2_point) * ((gt_P_C2_point(3)<0)*(-1) + (gt_P_C2_point(3)>=0)* 1);
    gt_P_C2_inv_signed = 1/ gt_P_C2_norm_signed;
    
    
    
    
    %% plot histogram of est_P_C2 signed radial distance
    figure('name',['histogram est_P_C2 signed radial distance ',funcHandle_name{funcHandleIdx}], 'NumberTitle', 'off')
    
    sign_z_est_P_C2 = (est_P_C2_pointSet(3,:)<0)*(-1) + (est_P_C2_pointSet(3,:)>=0)* 1;
    est_P_C2_radialdist_signed = sign_z_est_P_C2.*column_norm(est_P_C2_pointSet);
    inv_est_P_C2_norm_signed = ones(size(est_P_C2_radialdist_signed))./est_P_C2_radialdist_signed;
    subplot(1,2,1)
    histogram(est_P_C2_radialdist_signed)
    hold on
    plot (gt_P_C2_norm_signed, 0, '*')
    text (gt_P_C2_norm_signed, 0, 'gt_P_C2 radialdist', 'Interpreter','none')
    hold off
    title({['histogram of est_P_C2 signed radial distance ',funcHandle_name{funcHandleIdx} ],...
        ['pointIdx ', num2str(pointIdx), ' frame ', num2str(frameIdx)]}, 'Interpreter','none')
    
    %% plot histogram of est_P_C2 signed inv radial distance
    subplot(1,2,2)
    histogram(inv_est_P_C2_norm_signed)
    hold on
    plot (gt_P_C2_inv_signed, 0, '*')
    text (gt_P_C2_inv_signed, 0, 'gt_P_C2 inv radialdist', 'Interpreter','none')
    hold off
    title({['histogram of est_P_C2 signed inverse radial distance ', funcHandle_name{funcHandleIdx} ],...
        ['pointIdx ', num2str(pointIdx), ' frame ', num2str(frameIdx)]}, 'Interpreter','none')
    
    
    %% histogram (class CHistogram) of est_P_C2 signed norm
    histo_radius_ = 100;
    CHistogram_Obj = CHistogram(100, 3, gt_P_C2_norm_signed-histo_radius_, gt_P_C2_norm_signed +histo_radius_, -1);
    CHistogram_Obj.update(est_P_C2_radialdist_signed, zeros(size(est_P_C2_radialdist_signed)) );
    CHistogram_Obj.plot(['est_P_C2 signed radial distance ', funcHandle_name{funcHandleIdx}, '(parallax angle =',num2str(rad2deg(ang_rad),'%10.5e'),')'] );
    hold on
    plot (gt_P_C2_norm_signed, 0, '*')
    text (gt_P_C2_norm_signed, 0, 'gt_P_C2 radialdist', 'Interpreter','none')
    hold off
    title({['histogram (CHistogram) of est_P_C2 signed radial distance ',funcHandle_name{funcHandleIdx} ],...
        ['pointIdx ', num2str(pointIdx), ' frame ', num2str(frameIdx)]}, 'Interpreter','none')

    %% histogram (class function) of est_P_C2 signed inv radial distance
    histo_radius_inv = 1;
    CHistogramObj_inv = CHistogram(200, 3, gt_P_C2_inv_signed-histo_radius_inv, gt_P_C2_inv_signed +histo_radius_inv, -1);
    CHistogramObj_inv.update(inv_est_P_C2_norm_signed, zeros(size(inv_est_P_C2_norm_signed)) );
    CHistogramObj_inv.plot(['est_P_C2 signed inv radial distance ', funcHandle_name{funcHandleIdx},'(parallax angle =',num2str(rad2deg(ang_rad),'%10.5e'),')'] );
    hold on
    plot (gt_P_C2_inv_signed, 0, '*')
    text (gt_P_C2_inv_signed, 0, 'gt_P_C2 inv radialdist', 'Interpreter','none')
    hold off
    title({['histogram (CHistogram) of est_P_C2 signed inverse radial distance', funcHandle_name{funcHandleIdx} ],...
        ['pointIdx ', num2str(pointIdx), ' frame ', num2str(frameIdx)]}, 'Interpreter','none')
    
    
    
end


end