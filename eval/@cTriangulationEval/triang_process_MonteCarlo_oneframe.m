function triangdata = triang_process_MonteCarlo_oneframe(rawdata, frameIdx, flowSimObj, funcHandlesTriang)



% numFramesToProcess = length(rawdata.camMotion);
numIteration = size(rawdata.sim_P_N1,1);
tooCloseToCamera_tolerance = 1;

% containers of the triangulated data
triangdata.est_P_C2 = cell(length(funcHandlesTriang),numIteration );
triangdata.dist_to_FoE = cell(length(funcHandlesTriang),numIteration );
triangdata.parallax_angle = cell(length(funcHandlesTriang),numIteration );
triangdata.error_z = cell(length(funcHandlesTriang),numIteration );
triangdata.error_invz = cell(length(funcHandlesTriang),numIteration );
triangdata.error_radialdist = cell(length(funcHandlesTriang),numIteration );
triangdata.invertedsign_z_flag = cell(length(funcHandlesTriang),numIteration );
triangdata.tooCloseToCamera_flag = cell(length(funcHandlesTriang),numIteration );

for iterationIdx = 1:1:numIteration
    sim_P_N1 = rawdata.sim_P_N1{iterationIdx,frameIdx};
    sim_P_N2 = rawdata.sim_P_N2{iterationIdx,frameIdx};
    gt_P_C2 = rawdata.gt_P_C2{1,frameIdx};
    sign_z_gt_P_C2 = 1*(gt_P_C2(3,:)>=0) + (-1)*(gt_P_C2(3,:)<0);
    signed_norm_gt_P_C2 = sign_z_gt_P_C2 .* column_norm(gt_P_C2);
    % parallax angle
    sim_P_N2_C1 = rawdata.camMotion{1,frameIdx}.R_C2C1' * sim_P_N2;
    ang_rad = angRadBetw2Rays(sim_P_N1, sim_P_N2_C1);
    
    sim_P_I2 = flowSimObj.simIntrinsics.projPinHoleN(sim_P_N2);
    dist_to_FoE = column_norm(sim_P_I2 - repmat(rawdata.FoE(:,frameIdx),1,size(sim_P_I2,2)));
    for funcHandleIdx = 1:1:length(funcHandlesTriang)
        % Triangulation
        est_P_C2 = funcHandlesTriang{funcHandleIdx}(sim_P_N1, sim_P_N2, rawdata.camMotion{1,frameIdx});
        % condition : a and b are of different signs, sign(a) + sign(b) = 0
        invertedsign_Z_indx = ( sign(est_P_C2(3,:)) + sign(gt_P_C2(3,:)) == 0);
        % the points which are too close to the camera within +/-tooCloseToCamera_tolerance
        tooCloseToCamera_indx = (abs(est_P_C2(3,:))< tooCloseToCamera_tolerance);
        % est_P_C2
        triangdata.est_P_C2{funcHandleIdx,iterationIdx} = est_P_C2;
        % distance to FoE (norm. image coordinates)
        triangdata.dist_to_FoE{funcHandleIdx,iterationIdx} = dist_to_FoE;
        % parallax angle    
        triangdata.parallax_angle{funcHandleIdx,iterationIdx} = rad2deg(ang_rad);
        % error of z
        triangdata.error_z{funcHandleIdx,iterationIdx} = est_P_C2(3,:)-gt_P_C2(3,:);
        % error of invz
        triangdata.error_invz{funcHandleIdx,iterationIdx} = ones(1, size(est_P_C2,2))./est_P_C2(3,:) - ones(1, size(gt_P_C2,2))./gt_P_C2(3,:);
        % error of radial distance (norm)
        sign_z_est_P_C2 = 1*(est_P_C2(3,:)>=0) + (-1)*(est_P_C2(3,:)<0);
        
        triangdata.error_radialdist{funcHandleIdx,iterationIdx} = sign_z_est_P_C2 .* column_norm(est_P_C2) - signed_norm_gt_P_C2;
        % inverted z flag : 1 inplies that the point has inverted sign in z
        triangdata.invertedsign_z_flag{funcHandleIdx,iterationIdx} = zeros(1,size(est_P_C2,2));
        triangdata.invertedsign_z_flag{funcHandleIdx,iterationIdx}(1,invertedsign_Z_indx) = 1;
        % tooCloseToCamera : 1 inplies that the point has z in the range between +/-tooCloseToCamera_tolerance
        triangdata.tooCloseToCamera_flag{funcHandleIdx,iterationIdx} = zeros(1,size(est_P_C2,2));
        triangdata.tooCloseToCamera_flag{funcHandleIdx,iterationIdx}(1,tooCloseToCamera_indx) = 1;
    end
    
end


% sim_P_N1 = cell2mat(rawdata.sim_P_N1(:,frameIdx)');
% sim_P_N2 = cell2mat(rawdata.sim_P_N2(:,frameIdx)');






end