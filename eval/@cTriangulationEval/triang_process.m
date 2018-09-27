%% function: triang_process
% author: <Lin, Wei> (CC-DA/ESV1)
% date: <2017-04-11>
%  do the triangulation and collect the result after the triangulation
% INPUTS
% 
% 
function triang_data = triang_process(obj, triangFuncHandles, triangFuncNames, variant)

assert(ismember(variant, {'gt', 'sim'}), 'Unkown variant!');

tooCloseToCameraToleranceZ = obj.params.tooCloseToCameraToleranceZ;

% use ground truth data or simulated data
P_N1_all = obj.raw_data.([variant, '_P_N1']);
P_N2_all = obj.raw_data.([variant, '_P_N2']);

numFrames = size(P_N1_all,2);
numMonteCarloIterations = size(P_N1_all,1);

triang_data.est_P_C2 = cell(numel(triangFuncHandles), numMonteCarloIterations, numFrames);
% triang_data.est_Std = cell(numel(triangFuncHandles), numMonteCarloIterations, numFrames);
triang_data.error_z = cell(numel(triangFuncHandles), numMonteCarloIterations, numFrames);
triang_data.error_invz = cell(numel(triangFuncHandles), numMonteCarloIterations, numFrames);
triang_data.error_radialdist = cell(numel(triangFuncHandles), numMonteCarloIterations, numFrames);

triang_data.error_3D = cell(numel(triangFuncHandles), numMonteCarloIterations, numFrames);
triang_data.error_2D = cell(numel(triangFuncHandles), numMonteCarloIterations, numFrames);
triang_data.invertedsign_z_flag = cell(numel(triangFuncHandles), numMonteCarloIterations, numFrames);
triang_data.tooCloseToCamera_flag = cell(numel(triangFuncHandles), numMonteCarloIterations, numFrames);
triang_data.outsideErrorTolerance_flag = cell(numel(triangFuncHandles), numMonteCarloIterations, numFrames); 
triang_data.funcNames = triangFuncNames;

triang_data.z_C2ZeroFlag = cell(numel(triangFuncHandles), numMonteCarloIterations, numFrames);

camMotionCell = obj.raw_data.(['camMotion_',variant]);
tic;
for frameIdx = 1 : numFrames % for all frames
    
    gt_P_C2 = obj.raw_data.gt_P_C2{1,frameIdx};
    gt_P_I2 = obj.raw_data.gt_P_I2{1,frameIdx};
    
    sign_radDist_gt = sign(gt_P_C2(3,:));
    signed_norm_gt_P_C2 = sign_radDist_gt .* column_norm(gt_P_C2);
    assert(nnz(signed_norm_gt_P_C2==0)==0, 'Simulated point at (0,0,0)!');
    
    for MCiterIdx = 1 : numMonteCarloIterations % for all MC iterations
    
        P_N1 = P_N1_all{MCiterIdx,frameIdx};
        P_N2 = P_N2_all{MCiterIdx,frameIdx};
        
        camMotion = camMotionCell{MCiterIdx,frameIdx};
        
        for funcHandleIdx = 1 : numel(triangFuncHandles) % for all triangulation formulas
            
            triangFuncHandle = triangFuncHandles{funcHandleIdx};
            
            % Triangulation
            [est_P_C2, ~, ~] = triangFuncHandle(P_N1, P_N2, camMotion);
            
            % flag of z=0 of est_P_C2
%             triang_data.z_C2ZeroFlag{funcHandleIdx,MCiterIdx,frameIdx} = (est_P_C2(3,:)==0);
            
            % est_P_C2
            triang_data.est_P_C2{funcHandleIdx,MCiterIdx,frameIdx} = est_P_C2;
            % triang_data.est_Std{funcHandleIdx,MCiterIdx,frameIdx} = est_Std;
            
%             assert(nnz(est_P_C2(3,:)==0)==0); % this will fail the later sign check!
            
            % inverted z flag : 1 inplies that the point has inverted sign in z
            % condition : a and b are of different signs
%             triang_data.invertedsign_z_flag{funcHandleIdx,MCiterIdx,frameIdx} = (sign(est_P_C2(3,:)) ~= sign(gt_P_C2(3,:)));
            
%             if obj.params.processingFlags.doCorrectInvertions
%                 % correct invertions of sign before calculating error
%                 est_P_C2(:,triang_data.invertedsign_z_flag{funcHandleIdx,MCiterIdx,frameIdx}) = -1.0 .* est_P_C2(:,triang_data.invertedsign_z_flag{funcHandleIdx,MCiterIdx,frameIdx});
%             end
%             
%             onesVec = ones(1,size(est_P_C2,2));
%             
%             % error of z
%             triang_data.error_z{funcHandleIdx,MCiterIdx,frameIdx} = est_P_C2(3,:) - gt_P_C2(3,:);
%             % error of inverse z
%             triang_data.error_invz{funcHandleIdx,MCiterIdx,frameIdx} = onesVec./est_P_C2(3,:) - onesVec./gt_P_C2(3,:);
%             % error of radial distance (norm)
%             sign_radDist_est = sign(est_P_C2(3,:));
%             signed_norm_est_P_C2 = sign_radDist_est .* column_norm(est_P_C2);
%             triang_data.error_radialdist{funcHandleIdx,MCiterIdx,frameIdx} = signed_norm_est_P_C2 - signed_norm_gt_P_C2;
%             % error of inverse radial distance (norm)
%             triang_data.error_invradialdist{funcHandleIdx,MCiterIdx,frameIdx} = onesVec./signed_norm_est_P_C2 - onesVec./signed_norm_gt_P_C2;
            
            % 2D error
            triang_data.error_2D{funcHandleIdx,MCiterIdx,frameIdx} = column_norm( obj.flowSimObj.simIntrinsics.projPinHole(est_P_C2) - gt_P_I2);
            % 3D error
            triang_data.error_3D{funcHandleIdx,MCiterIdx,frameIdx} = column_norm(est_P_C2 - gt_P_C2);
            
            % error of inverse radial distance (norm)
            %triang_data.inv_error_radialdist{funcHandleIdx,j,i} = onesVec ./ (signed_norm_est_P_C2-signed_norm_gt_P_C2);
            
            %triang_data.error_rmse_radialdist{funcHandleIdx,j,i} = sqrt(mean((signed_norm_gt_P_C2 - signed_norm_est_P_C2).^2));  % Root Mean Squared Error
            
            
            % tooCloseToCamera : 1 inplies that the point has z in the range between +/-tooCloseToCamera_tolerance
            % the points which are too close to the camera within +/-tooCloseToCamera_tolerance
%             triang_data.tooCloseToCamera_flag{funcHandleIdx,MCiterIdx,frameIdx} = (abs(est_P_C2(3,:)) < tooCloseToCameraToleranceZ);
            
            % find points with z outside of tollerated result range
%             triang_data.aboveErrorTolerance_flag{funcHandleIdx,MCiterIdx,frameIdx} = abs(est_P_C2(3,:)) > obj.params.errorToleranceZ;
           
        end
    end
end
t_triang = toc

end