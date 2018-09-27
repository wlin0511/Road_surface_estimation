%% function: update_container(no used anymore)
% author: <Lin, Wei> (CC-DA/ESV1)
% date: <2017-04-11>
% update the container of gt_P_C, sim_P_N (collection of the raw data)
% INPUTS
% camMotion : the structure which carries camera motion data
% P_N1, P_N2 : points from optical flow of sequence frame data


function update_container___(obj, camMotion, P_N1, P_N2, gtPointCloudPlotFigHandle, frameIdx )
% camMotion
obj.raw_data.camMotion{1, frameIdx} = camMotion;

% compute the FoE
t_C2C1_N2 = [camMotion.t_C2C1_N2;1];
obj.raw_data.FoE(:,frameIdx) = obj.flowSimObj.simIntrinsics.projPinHole(t_C2C1_N2);

% for each frame, compute and save the data in the containers
simTraveledDist = norm(camMotion.t_C2C1_C2);
% Set translation scale to given input
t_C2C1_C2 = vec2unitVec(camMotion.t_C2C1_C2) * simTraveledDist;
% Triangulate 3D points in C1 for given flow field and camera motion,
% consider the triangulation result as the ground truth value
[gt_P_C1, ~] = obj.flowSimObj.triangulatePointsFromFlow(camMotion.R_C2C1, camMotion.t_C2C1_C2, P_N1, P_N2, gtPointCloudPlotFigHandle);
% Get point cloud in second view
gt_P_C2 = transformFromFrame_EUC_R(gt_P_C1, camMotion.R_C2C1, t_C2C1_C2);

std_N = obj.flowSimObj.std_N;  %  obj.flowSimObj.std_N = obj.flowSimObj.simIntrinsics.invProjPinHoleVec(obj.flowSimObj.params.std_I);
% 0 noise to gt_P_N1, if neither windscreen distortion nor lens distortion, then sim_P_N1 = gt_P_N1 
[sim_P_N1, gt_P_N1] = obj.flowSimObj.getImageOfPointCloud(gt_P_C1, [0;0]); % gt_P_N1 = C2N(gt_P_C1);
% add noise of std_N to gt_P_N2 (end point)   and    check whether the projection of sim_P_N is on the image
[sim_P_N2, gt_P_N2, validPointsIdx] = obj.flowSimObj.getImageOfPointCloud(gt_P_C2, std_N); % gt_P_N2 = C2N(gt_P_C2);

% get rid of the invalid flow
obj.raw_data.gt_P_C1{1,frameIdx} = gt_P_C1(:,validPointsIdx);
obj.raw_data.gt_P_C2{1,frameIdx} = gt_P_C2(:,validPointsIdx);
obj.raw_data.gt_P_N1{1,frameIdx} = gt_P_N1(:,validPointsIdx);
obj.raw_data.gt_P_N2{1,frameIdx} = gt_P_N2(:,validPointsIdx);
obj.raw_data.sim_P_N1{1,frameIdx} = sim_P_N1(:,validPointsIdx);
obj.raw_data.sim_P_N2{1,frameIdx} = sim_P_N2(:,validPointsIdx);

end