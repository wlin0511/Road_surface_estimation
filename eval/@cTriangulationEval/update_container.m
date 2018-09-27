%% function: update_container
% author: <Lin, Wei> (CC-DA/ESV1)
% date: <2017-04-21>
% update the container of gt_P_C, gt_P_N and sim_P_N in one frame
% works both with or without MonteCarlo simulation fo
% INPUTS
% camMotion : the structure which carries camera motion data
% P_N1, P_N2 : points from optical flow of sequence frame data
function update_container(obj, camMotion, P_N1, P_N2, frameIdx, sequFrameNb)

numIteration = size(obj.raw_data.sim_P_N1,1); % if it is not MonteCarlo Simulation, then numIteration = 1

% sequFrameNb
obj.raw_data.sequFrameNb(1,frameIdx) = sequFrameNb;

% % compute the FoE
% obj.raw_data.FoE(:,frameIdx) = obj.flowSimObj.simIntrinsics.projPinHole(camMotion.t_C2C1_N2);

%% for each frame, compute and save the data in the containers

% Triangulate 3D points in C1 for given flow field and camera motion,
% consider the triangulation result as the ground truth value
[gt_P_C1, ~, lowParallaxPointsLogic] = obj.flowSimObj.triangulatePointsFromFlow(camMotion.R_C2C1, camMotion.t_C2C1_C2, P_N1, P_N2);
% Get point cloud in second view
gt_P_C2 = transformFromFrame_EUC_R(gt_P_C1, camMotion.R_C2C1, camMotion.t_C2C1_C2);

std_N = obj.flowSimObj.std_N;  %  obj.flowSimObj.std_N = obj.flowSimObj.simIntrinsics.invProjPinHoleVec(obj.flowSimObj.params.std_I);
 
obj.raw_data.lowParallaxFlag{1,frameIdx} = lowParallaxPointsLogic;
obj.raw_data.gt_P_C1{1,frameIdx} = gt_P_C1;
obj.raw_data.gt_P_C2{1,frameIdx} = gt_P_C2;
gt_P_N1 = C2N(gt_P_C1);
gt_P_N2 = C2N(gt_P_C2);
obj.raw_data.gt_P_N1{1,frameIdx} = gt_P_N1;
obj.raw_data.gt_P_N2{1,frameIdx} = gt_P_N2;

invalidPointsIdx = cell(1, numIteration);
invalidPointsIdx_union = [];

% camMotion_gt
obj.raw_data.camMotion_gt{1,frameIdx} = camMotion;

% distance to FoE of gt_P_I2 (norm. image coordinates)
gt_P_I2 = obj.flowSimObj.simIntrinsics.projPinHoleN(gt_P_N2);
obj.raw_data.gt_P_I2{1,frameIdx} = obj.flowSimObj.simIntrinsics.projPinHole(gt_P_C2);
foe_I_gt = obj.flowSimObj.simIntrinsics.projPinHoleN(camMotion.t_C2C1_N2);
obj.raw_data.dist_to_FoE_gt{1,frameIdx} = column_norm(gt_P_I2 - repmat(foe_I_gt,1,size(gt_P_I2,2)));

% parallax angle between gt_P_N1 and gt_P_N2
gt_P_N2_C1 = camMotion.R_C2C1' * gt_P_N2;
ang_rad_gt = angRadBetw2Rays(gt_P_N1, gt_P_N2_C1);
obj.raw_data.parallax_angle_gt{1,frameIdx} = rad2deg(ang_rad_gt);


for j = 1 : numIteration
     
    [sim_P_N1, sim_P_N2, ~, ~, ~, ~, validPointsIdx, noiseMagnitude] = obj.flowSimObj.getFlowOfPointCloud(gt_P_C1, camMotion.R_C2C1, camMotion.t_C2C1_C2, std_N);
    
    invalidPointsIdx{1,j} = setdiff(1:size(sim_P_N1,2), validPointsIdx); 
    invalidPointsIdx_union = union(invalidPointsIdx_union,invalidPointsIdx{1,j} );
    
    obj.raw_data.sim_P_N1{j,frameIdx} = sim_P_N1;
    obj.raw_data.sim_P_N2{j,frameIdx} = sim_P_N2;
    obj.raw_data.noiseMagnitude{j, frameIdx} = noiseMagnitude;
    
    % camMotion_sim
    % noise in camera motion
%     obj.raw_data.camMotion_sim{j,frameIdx} = obj.simulateErroneousCamMotion(camMotion);

    % no noise in camera motion
    obj.raw_data.camMotion_sim{j,frameIdx} = camMotion;

    % distance to FoE of sim_P_I2 (norm. image coordinates)   todo: derotate!
    sim_P_I2 = obj.flowSimObj.simIntrinsics.projPinHoleN(sim_P_N2);
    foe_I_sim = obj.flowSimObj.simIntrinsics.projPinHoleN(obj.raw_data.camMotion_sim{j,frameIdx}.t_C2C1_N2);
    obj.raw_data.dist_to_FoE_sim{j,frameIdx} = column_norm(sim_P_I2 - repmat(foe_I_sim,1,size(sim_P_I2,2)));
    %
    obj.raw_data.dist_to_FoE_sim{j,frameIdx} = obj.raw_data.dist_to_FoE_gt{1,frameIdx}; % gt foe dist is used
    
    % parallax angle between sim_P_N1 and sim_P_N2
    %sim_P_N2_C1 = obj.raw_data.camMotion_sim{j,frameIdx}.R_C2C1' * sim_P_N2;
    %ang_rad_sim = angRadBetw2Rays(sim_P_N1, sim_P_N2_C1);
    %obj.raw_data.parallax_angle_sim{j,frameIdx} = rad2deg(ang_rad_sim);
    
    
    % gt parallax is used for each iteration
    obj.raw_data.parallax_angle_sim{j,frameIdx} =  obj.raw_data.parallax_angle_gt{1,frameIdx}; 
    
    
   
end

%% Remove any data that was invalid simulated at least one time

% go again over all simulated flow fields of MC simulation and remove the
% simulated flow vectors 
for j = 1 : numIteration
    % remove the invalid points of common invalidPointsIdx in all the iterations
    obj.raw_data.sim_P_N1{j, frameIdx}(:, invalidPointsIdx_union) = [];
    obj.raw_data.sim_P_N2{j, frameIdx}(:, invalidPointsIdx_union) = [];
    obj.raw_data.noiseMagnitude{j, frameIdx}(:, invalidPointsIdx_union) = [];
    obj.raw_data.dist_to_FoE_sim{j,frameIdx}(invalidPointsIdx_union) = [];
    obj.raw_data.parallax_angle_sim{j,frameIdx}(invalidPointsIdx_union) = [];
end

% remove the points of invalidPointsIdx (for sim_P_N1 and sim_P_N2) in
% gt_P_C1, gt_P_C2, gt_P_N1 and gt_P_N2 for the sake of correspondence
obj.raw_data.gt_P_C1{1,frameIdx}(:, invalidPointsIdx_union) = [];
obj.raw_data.gt_P_C2{1,frameIdx}(:, invalidPointsIdx_union) = [];
obj.raw_data.gt_P_I2{1,frameIdx}(:, invalidPointsIdx_union) = [];
obj.raw_data.gt_P_N1{1,frameIdx}(:, invalidPointsIdx_union) = [];
obj.raw_data.gt_P_N2{1,frameIdx}(:, invalidPointsIdx_union) = [];
obj.raw_data.parallax_angle_gt{1,frameIdx}(invalidPointsIdx_union) = [];
obj.raw_data.lowParallaxFlag{1,frameIdx}(invalidPointsIdx_union) = [];


end
