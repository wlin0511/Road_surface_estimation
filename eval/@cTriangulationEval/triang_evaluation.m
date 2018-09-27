%% function: triang_evaluation ( not finished )
% author: <Lin, Wei> (CC-DA/ESV1)
% date: <2017-03-17>
% the function for triangulation evaluation
% INPUTS
% cTriangEvalObj : object of the class cTriangulationEval
% img : image in the sequence frame data
% camMotion : the structure which carries camera motion data
% intrinsicObj : intrinsicObj from sequence frame data
% P_N1, P_N2 : points from optical flow of sequence frame data
% variant : 'multi_frame' : evaluation based on the data of all the frames
%                           in one iteration
%           'one_frame' : evaluation based on the data of one frame in
%                        multiple iterations
function evaldata_multi_frame = triang_evaluation(obj, img,camMotion, intrinsicObj,P_N1, P_N2, gtPointCloudPlotFigHandle, variant,sequFrameNb, framesIdxToProcess,evaldata_multi_frame )

t_C2C1_N2 = [camMotion.t_C2C1_N2;1];
FoE =intrinsicObj.projPinHole(t_C2C1_N2);
tooCloseToCamera_tolerance = 1;
switch variant
    case 'multi_frame'
        numFramesToProcess = length(framesIdxToProcess);
        frameIdx = find(framesIdxToProcess == sequFrameNb);
        
        % for each frame, compute and save the data in the containers
        simTraveledDist = norm(camMotion.t_C2C1_C2);
        % Set translation scale to given input
        t_C2C1_C2 = vec2unitVec(camMotion.t_C2C1_C2) * simTraveledDist;
        % Triangulate 3D points in C1 for given flow field and camera motion
        [gt_P_C1, triangulatedPointsIdx] = obj.flowSimObj.triangulatePointsFromFlow(camMotion.R_C2C1, camMotion.t_C2C1_C2, P_N1, P_N2, gtPointCloudPlotFigHandle);
        % Get point cloud in second view
        gt_P_C2 = transformFromFrame_EUC_R(gt_P_C1, camMotion.R_C2C1, t_C2C1_C2);
        
        std_N = obj.flowSimObj.std_N;
        % add noise to correspondences end point
        [sim_P_N1, gt_P_N1] = obj.flowSimObj.getImageOfPointCloud(gt_P_C1, [0;0]); % gt_P_N1 = C2N(gt_P_C1);
        % check whether the projection of sim_P_N is on the image
        [sim_P_N2, gt_P_N2, validPointsIdx] = obj.flowSimObj.getImageOfPointCloud(gt_P_C2, std_N); % gt_P_N2 = C2N(gt_P_C2);
        
        % get rid of the invalid flow
        sim_P_N1 = sim_P_N1(:,validPointsIdx);
        sim_P_N2 = sim_P_N2(:,validPointsIdx);
        gt_P_N1 = gt_P_N1(:,validPointsIdx);
        gt_P_N2 = gt_P_N2(:,validPointsIdx);
        gt_P_C1 = gt_P_C1(:,validPointsIdx);
        gt_P_C2 = gt_P_C2(:,validPointsIdx);
        
        simulatedPointsIdx = triangulatedPointsIdx(validPointsIdx);
        
        sim_P_I2 = intrinsicObj.projPinHoleN(sim_P_N2);
        dist_to_FoE = column_norm(sim_P_I2 - repmat(FoE,1,size(sim_P_I2,2)));
        for i = 1:1:length(obj.funcHandlesTriang)
            % Triangulation
            est_P_C2 = obj.funcHandlesTriang{i}(sim_P_N1, sim_P_N2, camMotion);
            % condition : a and b are of different signs, sign(a) + sign(b) = 0
            invertedsign_Z_indx = ( sign(est_P_C2(3,:)) + sign(gt_P_C2(3,:)) == 0);
            % the points which are too close to the camera within +/-tooCloseToCamera_tolerance
            tooCloseToCamera_indx = (abs(est_P_C2(3,:))< tooCloseToCamera_tolerance);
            evaldata_multi_frame.dist_to_FoE{i,frameIdx}(1,:) = dist_to_FoE; % distance to FoE (norm. image coordinates)
            sim_P_N2_C1 = camMotion.R_C2C1' * sim_P_N2;
            ang_rad = angRadBetw2Rays(sim_P_N1, sim_P_N2_C1);
            evaldata_multi_frame.parallax_angle{i,frameIdx}(1,:) = rad2deg(ang_rad);
            evaldata_multi_frame.error_z{i,frameIdx}(1,:) = est_P_C2(3,:)-gt_P_C2(3,:); % error of z
            evaldata_multi_frame.error_invz{i,frameIdx}(1,:) = ones(1, size(est_P_C2,2))./est_P_C2(3,:) - ones(1, size(gt_P_C2,2))./gt_P_C2(3,:); % error of inverse z
            sign_z_est_P_C2 = 1*(est_P_C2(3,:)>=0) + (-1)*(est_P_C2(3,:)<0);
            sign_z_gt_P_C2 = 1*(gt_P_C2(3,:)>=0) + (-1)*(gt_P_C2(3,:)<0);
            evaldata_multi_frame.error_radialdist{i,frameIdx}(1,:) = sign_z_est_P_C2 .* column_norm(est_P_C2) - sign_z_gt_P_C2 .* column_norm(gt_P_C2); % error of radial distance (norm)
            evaldata_multi_frame.invertedsign_z_flag{i,frameIdx}(1,:) = zeros(1,size(est_P_C2,2)); % 1 inplies that this point has inverted sign in z
            evaldata_multi_frame.invertedsign_z_flag{i,frameIdx}(1,invertedsign_Z_indx) = 1;
            evaldata_multi_frame.tooCloseToCamera_flag{i,frameIdx}(1,:) = zeros(1,size(est_P_C2,2)); % 1 inplies that this point has z in range between +/-tooCloseToCamera_tolerance
            evaldata_multi_frame.tooCloseToCamera_flag{i,frameIdx}(1,tooCloseToCamera_indx) = 1;
        end
        
        if frameIdx == numFramesToProcess % last frame, data processing
            for i = 1:1:length(obj.funcHandlesTriang)
                
                [~, parallax_angle_mat, error_invz_mat, error_z_mat,error_radialdist_mat,invertedsign_z_flag_mat,tooCloseToCamera_flag_mat] = plot_evaldata(evaldata_multi_frame, i);
                error_invz_tolerance = 0.02;
                error_z_tolerance = 500;
                error_radialdist_tolerance = 1000;
                % number of outliers
                num_error_invz_outliers = sum(abs(error_invz_mat)>=error_invz_tolerance);
                num_error_z_outliers = sum(abs(error_z_mat)>=error_z_tolerance);
                num_error_radialdist_outliers = sum(abs(error_radialdist_mat)>=error_radialdist_tolerance);
                % number of total not-NaN data
                num_total_not_nan_data = sum(~isnan(error_invz_mat));
                % number of NaN or Inf
                num_notfinite_data = sum(~isfinite(error_invz_mat));
                % number of inverted sign of z
                num_invertedsign_z = sum(invertedsign_z_flag_mat);
                num_tooCloseToCamera = sum(tooCloseToCamera_flag_mat);
                % plot the frequency map error of invz - parallax angle
                plot_frequency_map(parallax_angle_mat,error_invz_mat,error_invz_tolerance)
                xlabel('parallax angle in [degree]')
                ylabel('error of inverse z in [m^{-1}]')
                title({['Error of inverse z vs. parallax angle triang\_funcHandle ',num2str(i)];...
                    [num2str(num_error_invz_outliers),' outliers of ',num2str(num_total_not_nan_data),' data with tolerance ',num2str(error_invz_tolerance)];...
                    [num2str(num_invertedsign_z),' inverted sign of z, ',num2str(num_notfinite_data),' NaN or Inf, ',...
                    num2str(num_tooCloseToCamera),' have z in the range of [-',num2str(tooCloseToCamera_tolerance),',+',num2str(tooCloseToCamera_tolerance),']']})
                % plot the frequency map error of z - parallax angle
                plot_frequency_map(parallax_angle_mat,error_z_mat,error_z_tolerance)
                xlabel('parallax angle in [degree]')
                ylabel('error of z in [m]')
                title({['Error of z vs. parallax angle triang\_funcHandle ',num2str(i)];...
                    [num2str(num_error_z_outliers),' outliers of ',num2str(num_total_not_nan_data),' data with tolerance ',num2str(error_z_tolerance)];...
                    [num2str(num_invertedsign_z),' inverted sign of z, ',num2str(num_notfinite_data),' NaN or Inf, ',...
                    num2str(num_tooCloseToCamera),' have z in the range of [-',num2str(tooCloseToCamera_tolerance),',+',num2str(tooCloseToCamera_tolerance),']']})
                % plot the frequency map error of radial distance - parallax angle
                plot_frequency_map(parallax_angle_mat,error_radialdist_mat,error_radialdist_tolerance)
                xlabel('parallax angle in [degree]')
                ylabel('error of radial distance in [m]')
                title({['Error of radial distance vs. parallax angle triang\_funcHandle ',num2str(i)];...
                    [num2str(num_error_radialdist_outliers),' outliers of ',num2str(num_total_not_nan_data),' data with tolerance ',num2str(error_radialdist_tolerance)];...
                    [num2str(num_invertedsign_z),' inverted sign of z, ',num2str(num_notfinite_data),' NaN or Inf, ',...
                    num2str(num_tooCloseToCamera),' have z in the range of [-',num2str(tooCloseToCamera_tolerance),',+',num2str(tooCloseToCamera_tolerance),']']})
                
%                 % plot the frequency map error of invz -
%                 % parallax angle with outliers
%                 plot_frequency_map(parallax_angle_mat,error_invz_mat)
%                 xlabel('parallax angle in [degree]')
%                 ylabel('error of inverse z in [m^{-1}]')
%                 title({['Error of inverse z vs. parallax angle with outliers triang\_funcHandle ',num2str(i)];...
%                     [num2str(num_total_not_nan_data),' not-NaN data'];...
%                     [num2str(num_invertedsign_z),' inverted sign of z, ',num2str(num_notfinite_data),' NaN or Inf, ',...
%                     num2str(num_tooCloseToCamera),' have z in the range of [-',num2str(tooCloseToCamera_tolerance),',+',num2str(tooCloseToCamera_tolerance),']']})
          
                parallax_angle_without_tooCloseToCamera = parallax_angle_mat(~tooCloseToCamera_flag_mat);
                error_invz_without_tooCloseToCamera = error_invz_mat(~tooCloseToCamera_flag_mat);
                error_z_without_tooCloseToCamera = error_z_mat(~tooCloseToCamera_flag_mat);
                error_radialdist_without_tooCloseToCamera = error_radialdist_mat(~tooCloseToCamera_flag_mat);
                % number of outliers
                num_error_invz_outliers_without_tooCloseToCamera = sum(abs(error_invz_without_tooCloseToCamera)>=error_invz_tolerance);
                num_error_z_outliers_without_tooCloseToCamera = sum(abs(error_z_without_tooCloseToCamera)>=error_z_tolerance);
                num_error_radialdist_outliers_without_tooCloseToCamera = sum(abs(error_radialdist_without_tooCloseToCamera)>=error_radialdist_tolerance);
                % number of total not-NaN data
                num_total_not_nan_data_without_tooCloseToCamera = sum(~isnan(error_invz_without_tooCloseToCamera));
                % number of inverted sign of z
                num_invertedsign_z_without_tooCloseToCamera = sum(invertedsign_z_flag_mat(~tooCloseToCamera_flag_mat));
                % number of NaN or Inf
                num_notfinite_data_without_tooCloseToCamera = sum(~isfinite(error_invz_without_tooCloseToCamera));
                % plot the frequency map error of invz - parallax angle ( without points with z in range [-1 +1])
                plot_frequency_map(parallax_angle_without_tooCloseToCamera,error_invz_without_tooCloseToCamera,error_invz_tolerance)
                xlabel('parallax angle in [degree]')
                ylabel('error of inverse z in [m^{-1}]')
                title({['Error of inverse z vs. parallax angle triang\_funcHandle ',num2str(i),' without z in the range of [-',num2str(tooCloseToCamera_tolerance),',+',num2str(tooCloseToCamera_tolerance),']'];...
                    [num2str(num_error_invz_outliers_without_tooCloseToCamera),' outliers of ',num2str(num_total_not_nan_data_without_tooCloseToCamera),' data with tolerance ',num2str(error_invz_tolerance)];...
                    [num2str(num_invertedsign_z_without_tooCloseToCamera),' inverted sign of z, ',num2str(num_notfinite_data_without_tooCloseToCamera ),' NaN or Inf, ',...
                    num2str(num_tooCloseToCamera),' have z in the range of [-',num2str(tooCloseToCamera_tolerance),',+',num2str(tooCloseToCamera_tolerance),']']})
                % plot the frequency map error of z - parallax angle ( without points with z in range [-1 +1])
                plot_frequency_map(parallax_angle_without_tooCloseToCamera,error_z_without_tooCloseToCamera,error_z_tolerance)
                xlabel('parallax angle in [degree]')
                ylabel('error of z in [m]')
                title({['Error of z vs. parallax angle triang\_funcHandle ',num2str(i),' without z in the range of [-',num2str(tooCloseToCamera_tolerance),',+',num2str(tooCloseToCamera_tolerance),']'];...
                    [num2str(num_error_z_outliers_without_tooCloseToCamera),' outliers of ',num2str(num_total_not_nan_data_without_tooCloseToCamera),' data with tolerance ',num2str(error_z_tolerance)];...
                    [num2str(num_invertedsign_z_without_tooCloseToCamera),' inverted sign of z, ',num2str(num_notfinite_data_without_tooCloseToCamera ),' NaN or Inf, ',...
                    num2str(num_tooCloseToCamera),' have z in the range of [-',num2str(tooCloseToCamera_tolerance),',+',num2str(tooCloseToCamera_tolerance),']']})
                % plot the frequency map error of radial distance - parallax angle ( without points with z in range [-1 +1])
                plot_frequency_map(parallax_angle_without_tooCloseToCamera,error_radialdist_without_tooCloseToCamera,error_radialdist_tolerance)
                xlabel('parallax angle in [degree]')
                ylabel('error of radial distance in [m]')
                title({['Error of radial distance vs. parallax angle triang\_funcHandle ',num2str(i),' without z in the range of [-',num2str(tooCloseToCamera_tolerance),',+',num2str(tooCloseToCamera_tolerance),']'];...
                    [num2str(num_error_radialdist_outliers_without_tooCloseToCamera),' outliers of ',num2str(num_total_not_nan_data_without_tooCloseToCamera),' data with tolerance ',num2str(error_radialdist_tolerance)];...
                    [num2str(num_invertedsign_z_without_tooCloseToCamera),' inverted sign of z, ',num2str(num_notfinite_data_without_tooCloseToCamera ),' NaN or Inf, ',...
                    num2str(num_tooCloseToCamera),' have z in the range of [-',num2str(tooCloseToCamera_tolerance),',+',num2str(tooCloseToCamera_tolerance),']']})
                
                
            end
        end
        
        
        
    case 'one_frame'
        iteration_num = 100;
        evaldata_one_frame.dist_to_FoE = cell(length(obj.funcHandlesTriang),iteration_num );
        evaldata_one_frame.parallax_angle = cell(length(obj.funcHandlesTriang),iteration_num );
        evaldata_one_frame.error_z = cell(length(obj.funcHandlesTriang),iteration_num );
        evaldata_one_frame.error_invz = cell(length(obj.funcHandlesTriang),iteration_num );
        evaldata_one_frame.error_radialdist = cell(length(obj.funcHandlesTriang),iteration_num );
        evaldata_one_frame.invertedsign_z_flag = cell(length(obj.funcHandlesTriang),iteration_num );
        
        for i = 1:1:length(obj.funcHandlesTriang)
            
            simTraveledDist = norm(camMotion.t_C2C1_C2);
            % Set translation scale to given input
            t_C2C1_C2 = vec2unitVec(camMotion.t_C2C1_C2) * simTraveledDist;
            % Triangulate 3D points in C1 for given flow field and camera motion
            [gt_P_C1, triangulatedPointsIdx] = obj.flowSimObj.triangulatePointsFromFlow(camMotion.R_C2C1, camMotion.t_C2C1_C2, P_N1, P_N2, gtPointCloudPlotFigHandle);
            % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Get optical flow from both views without simulation of rolling shutter
            % Get point cloud in second view
            gt_P_C2 = transformFromFrame_EUC_R(gt_P_C1, camMotion.R_C2C1, t_C2C1_C2);
            
            % triangulate 3D points in C1 and C2 and plot 3D points
            % [gt_P_C1, gt_P_C2, triangulatedPointsIdx] = cTriangEvalObj.flowSimObj.triangulatePointsFromFlow(camMotion, P_N1, P_N2,gtPointCloudPlotFigHandle);
            
            std_N = obj.flowSimObj.std_N;
            % std_N = [0;0];
            for j = 1:1:iteration_num
                % add noise to correspondences end point
                [sim_P_N1, gt_P_N1] = obj.flowSimObj.getImageOfPointCloud(gt_P_C1, [0;0]); % gt_P_N1 = C2N(gt_P_C1);
                [sim_P_N2, gt_P_N2, validPointsIdx] = obj.flowSimObj.getImageOfPointCloud(gt_P_C2, std_N); % gt_P_N2 = C2N(gt_P_C2);
                % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                % Get optical flow from both views without simulation of rolling shutter
                %[sim_P_N1, sim_P_N2, gt_P_N1, gt_P_N2, gt_P_C2, gt_P_C1, validPointsIdx] = cTriangEvalObj.flowSimObj.getFlowOfPointCloud(gt_P_C1, camMotion.R_C2C1, t_C2C1_C2, cTriangEvalObj.flowSimObj.std_N);
                
                % check whether the projection of sim_P_N is on the image
                % set the outliers as NaN
                sim_P_N1(:,setdiff(1:size(sim_P_N1,2), validPointsIdx )) = NaN;
                sim_P_N2(:,setdiff(1:size(sim_P_N2,2), validPointsIdx )) = NaN;
                gt_P_N1(:,setdiff(1:size(gt_P_N1,2), validPointsIdx )) = NaN;
                gt_P_N2(:,setdiff(1:size(gt_P_N2,2), validPointsIdx )) = NaN;
                gt_P_C1(:,setdiff(1:size(gt_P_C1,2), validPointsIdx )) = NaN;
                gt_P_C2(:,setdiff(1:size(gt_P_C2,2), validPointsIdx )) = NaN;
                
                simulatedPointsIdx = triangulatedPointsIdx(validPointsIdx);
                
                %[sim_P_N1, sim_P_N2, gt_P_N1, gt_P_N2,gt_P_C2, gt_P_C1, simulatedPointsIdx] = cTriangEvalObj.flowSimObj.reSimulateOpticalFlow(camMotion.R_C2C1, camMotion.t_C2C1_C2, P_N1, P_N2);
                
                sim_P_I2 = intrinsicObj.projPinHoleN(sim_P_N2);
                
                %  P_I2_used = P_I2(:,simulatedPointsIdx);
                
                dist_to_FoE = column_norm(sim_P_I2 - repmat(FoE,1,size(sim_P_I2,2)));
                est_P_C2 = obj.funcHandlesTriang{i}(sim_P_N1, sim_P_N2, camMotion);
                
                % condition : a and b are of different signs, sign(a) + sign(b) = 0
                invertedsign_Z_indx = ( sign(est_P_C2(3,:)) + sign(gt_P_C2(3,:)) == 0);
                % est_P_C2_invertedsign_Z = est_P_C2(:,invertedsign_Z_indx);
                % z_err{i,j}(1,:) = dist_to_FoE; % distance to FoE (norm. image coordinates)
                evaldata_one_frame.dist_to_FoE{i,j}(1,:) = dist_to_FoE; % distance to FoE (norm. image coordinates)
                %                 gt_P_N2_C1 = camMotion.R_C2C1' * gt_P_N2;
                %                 ang_rad = angRadBetw2Rays(gt_P_N1, gt_P_N2_C1);
                %                 eval_data.parallax_angle{i,j}(1,:) = rad2deg(ang_rad);
                sim_P_N2_C1 = camMotion.R_C2C1' * sim_P_N2;
                ang_rad = angRadBetw2Rays(sim_P_N1, sim_P_N2_C1);
                evaldata_one_frame.parallax_angle{i,j}(1,:) = rad2deg(ang_rad);
                
                evaldata_one_frame.error_z{i,j}(1,:) = est_P_C2(3,:)-gt_P_C2(3,:); % error of z
                evaldata_one_frame.error_invz{i,j}(1,:) = ones(1, size(est_P_C2,2))./est_P_C2(3,:) - ones(1, size(gt_P_C2,2))./gt_P_C2(3,:); % error of inverse z
                sign_z_est_P_C2 = 1*(est_P_C2(3,:)>=0) + (-1)*(est_P_C2(3,:)<0);
                sign_z_gt_P_C2 = 1*(gt_P_C2(3,:)>=0) + (-1)*(gt_P_C2(3,:)<0);
                evaldata_one_frame.error_radialdist{i,j}(1,:) = sign_z_est_P_C2 .* column_norm(est_P_C2) - sign_z_gt_P_C2 .* column_norm(gt_P_C2); % error of radial distance (norm)
                evaldata_one_frame.invertedsign_z_flag{i,j}(1,:) = zeros(1,size(est_P_C2,2)); % 1 inplies that this point has inverted sign in z
                evaldata_one_frame.invertedsign_z_flag{i,j}(1,invertedsign_Z_indx) = 1;
                
            end  % end of iterations
            
            [dist_to_FoE_mat, parallax_angle_mat, error_invz_mat] = plot_evaldata(evaldata_one_frame, i);
            
            %                         % logarithmic plot of error of invz - parallax angle
            %                         figure, scatter(parallax_angle_mat(1,error_invz_inlier_indx & non_invertedsign_z_indx),error_invz_mat(1,error_invz_inlier_indx & non_invertedsign_z_indx),'.');
            %                         hold on
            %                         scatter(parallax_angle_mat(1,error_invz_inlier_indx & invertedsign_z_indx),error_invz_mat(1,error_invz_inlier_indx & invertedsign_z_indx),'r','.')
            %                         set(gca, 'xscale', 'log')
            %                         xlabel('parallax angle')
            %                         ylabel('error of invz')
            %                         title('logarithmic plot of error of invz - parallax angle')
            
            %                         figure, histogram(parallax_angle_mat,'NumBins',1000,'BinLimits',[0 0.1])
            %                         title('histogram of parallax-angle 0-0.1')
            %                         figure, histogram(parallax_angle_mat,'NumBins',1000,'BinLimits',[0.1 0.2])
            %                         title('histogram of parallax-angle 0.1-0.2')
            %                         figure, histogram(parallax_angle_mat,'NumBins',1000,'BinLimits',[0.2 0.3])
            %                         title('histogram of parallax-angle 0.2-0.3')
            
            % plot the frequency map
            plot_frequency_map(dist_to_FoE_mat,error_invz_mat,0.2)
            xlabel('dist to FoE')
            ylabel('error of invz')
            title(['triang funcHandle ',num2str(i)])
            
            % among all the iterations, if a point ever has an inverted sign result, mark it as invertedsign;
            invertedsign_z_flag_mat = cell2mat(evaldata_one_frame.invertedsign_z_flag');
            any_invertedsign_z = any(invertedsign_z_flag_mat);
            
            gt_P_I1_non_invertedsign = intrinsicObj.projPinHoleN(gt_P_N1(:,(any_invertedsign_z==0)));
            gt_P_I2_non_invertedsign = intrinsicObj.projPinHoleN(gt_P_N2(:,(any_invertedsign_z==0)));
            gt_P_I1_invertedsign = intrinsicObj.projPinHoleN(gt_P_N1(:,(any_invertedsign_z==1)));
            gt_P_I2_invertedsign = intrinsicObj.projPinHoleN(gt_P_N2(:,(any_invertedsign_z==1)));
            
            figure, imshow(im2double(img),[])
            hold on
            scatter(FoE(1),FoE(2),'r','filled')
            plotLines(gt_P_I1_non_invertedsign,gt_P_I2_non_invertedsign,'g')
            plotLines(gt_P_I1_invertedsign,gt_P_I2_invertedsign,'r')
            title('non-inverted-signed:green inverted-signed:red in all iterations')
            
            error_invz_mat_ = cell2mat(evaldata_one_frame.error_invz');
            error_invz_mean = mean(error_invz_mat_);
            error_invz_std = std(error_invz_mat_);
            outlier_tolerance = 0.05;
            figure,histogram(error_invz_std,'BinLimits',[0 outlier_tolerance])
            title('error-invz-std')
            % filter out the outliers for a better visualization
            error_invz_std_inliers = error_invz_std;
            error_invz_std_inliers(error_invz_std_inliers > outlier_tolerance) = NaN;
            
            % plot the optical flow with varying color representing varying
            % standard deviation of invz error (with outliers)
            img_double = im2double(img)/max(max(im2double(img)));
            figure, imshow(repmat(img_double,1,1,3))
            hold on
            scatter(FoE(1),FoE(2),'r','filled')
            plotColorLines(intrinsicObj.projPinHoleN(gt_P_N1), intrinsicObj.projPinHoleN(gt_P_N2), error_invz_std_inliers)
            colorbar
            hold on
            plotLines(intrinsicObj.projPinHoleN(gt_P_N1(:,(error_invz_std > outlier_tolerance ))),intrinsicObj.projPinHoleN(gt_P_N2(:,( error_invz_std > outlier_tolerance ))),'r')
            title('std-dev among all the iterations  red: outliers')
            % varying color standard deviation of invz error (without outliers)
            figure, imshow(repmat(img_double,1,1,3))
            hold on
            scatter(FoE(1),FoE(2),'r','filled')
            plotColorLines(intrinsicObj.projPinHoleN(gt_P_N1), intrinsicObj.projPinHoleN(gt_P_N2), error_invz_std_inliers)
            colorbar
            title('std-dev among all the iterations without outliers')
            
        end  % end of for-loop for triang-function handles
end  % end of switch
end % end of function triang_evaluation