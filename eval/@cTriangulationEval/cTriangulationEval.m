classdef cTriangulationEval < handle
    properties
%         funcHandlesTriang   % cell of triangulation function handles of triangulation 
%         funcHandle_name     % cell of names of function handles of triangulation
%         triangErrHistos     % cell of histograms (CHistogram) representing the distribution of the triangluation errors
        %simStdI             % standard deviation to be used for simulation of point correspondences
        flowSimObj          % object of type cFlowSimulator used for simulation of point correspondences
        raw_data            % container of the raw data : gt_P_C and sim_P_N
        triang_data_sim         % container of the result of triangulation and data for plots 
        triang_data_gt      % container of the result of triangulation of gt_P_N1 and gt_P_N2
        %triang_data_MonteCarlo_oneframe
        
        params % .numMonteCarloIterations, .numFramesToProcess
    end
    
    methods
        
        function obj = cTriangulationEval( simIntrinsics, paramsTriangEval, paramsFlowSim)  
            obj.params = paramsTriangEval;
            
            % simIntrinsics : intrinsics object used in simulation
            % params : all parameters needed to simulate flow
            obj.flowSimObj = cFlowSimulator(simIntrinsics, paramsFlowSim);
            
            % initialize containers of the raw data
            obj.raw_data.sequFrameNb = zeros(1, obj.params.numFramesToProcess);
            obj.raw_data.camMotion_gt = cell(1, obj.params.numFramesToProcess );
            obj.raw_data.camMotion_sim = cell(obj.params.numMonteCarloIterations, obj.params.numFramesToProcess);
            
            obj.raw_data.lowParallaxFlag = cell(1, obj.params.numFramesToProcess );
            obj.raw_data.gt_P_C1 = cell(1, obj.params.numFramesToProcess );
            obj.raw_data.gt_P_C2 = cell(1, obj.params.numFramesToProcess );
            obj.raw_data.gt_P_N1 = cell(1, obj.params.numFramesToProcess );
            obj.raw_data.gt_P_N2 = cell(1, obj.params.numFramesToProcess );
            obj.raw_data.gt_P_I2 = cell(1, obj.params.numFramesToProcess );
            obj.raw_data.sim_P_N1 = cell(obj.params.numMonteCarloIterations, obj.params.numFramesToProcess );
            obj.raw_data.sim_P_N2 = cell(obj.params.numMonteCarloIterations, obj.params.numFramesToProcess );
            obj.raw_data.noiseMagnitude = cell(obj.params.numMonteCarloIterations, obj.params.numFramesToProcess );
            obj.raw_data.dist_to_FoE_gt = cell( 1, obj.params.numFramesToProcess );
            obj.raw_data.dist_to_FoE_sim = cell(obj.params.numMonteCarloIterations, obj.params.numFramesToProcess );
            
            obj.raw_data.parallax_angle_gt = cell( 1, obj.params.numFramesToProcess );
            obj.raw_data.parallax_angle_sim = cell(obj.params.numMonteCarloIterations, obj.params.numFramesToProcess );
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
%         % Updates the error histograms with one more simulated flow filed based
%         % on P_C1 and the given camera motion
%         function updateFromPointCloud(obj, P_C1, camMotion)
%             [sim_P_N1, sim_P_N2, gt_P_N1, gt_P_N2, gt_P_C2, gt_P_C1, validPointsIdx] = obj.flowSimObj.getFlowOfPointCloud(P_C1, camMotion.R_C2C1, camMotion.t_C2C1_C2); 
%                       
%             obj.updateHisto(sim_P_N1, sim_P_N2, gt_P_C2,camMotion);
%             
%         end
%         
%         % Updates the error histograms with one more simulated flow filed based
%         % on given point correspondences and camera motion
%         function updateFromCorrespondences(obj, P_N2, P_N1, camMotion)
%             % to get all the triangulation functions to have the same inputs
%             % you could pass instead of R_C2C2, t_ ... only an object camMotion
%             % that then has all the needed inputs as fields.
%             % this way we could for example just pass the tveObj from the frameDataObj
%            [sim_P_N1, sim_P_N2, gt_P_N1, gt_P_N2,gt_P_C2, gt_P_C1, simulatedPointsIdx] = obj.flowSimObj.reSimulateOpticalFlow(camMotion.R_C2C1, camMotion.t_C2C1_C2, P_N1, P_N2);
%                                  
%            obj.updateHisto(sim_P_N1, sim_P_N2, gt_P_C2,camMotion);           
%         end
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         
%         % Update histogram with idx histoIdx with current triangulation
%         % error
%         function updateHisto(obj, sim_P_N1, sim_P_N2, gt_P_C2,camMotion)
%             % todo
%             for i = 1:1:length(obj.funcHandlesTriang) % for all funcHandles
%               % est_P_C2 = triangulate again sim_P_N2, sim_P_N1, R_C2C1, t_C2C1_C2
%               
%               %[P_C1, triangluatedPointIdx] = obj.flowSimObj.triangulatePointsFromFlow(R_C2C1, t_C2C1_C2, sim_P_N1, sim_P_N2);
%               est_P_C2 = obj.funcHandlesTriang{i}(sim_P_N1, sim_P_N2, camMotion);
%               P_C2_diff = est_P_C2 - gt_P_C2;
%               P_C2_norm_err = column_norm(P_C2_diff);  % norm of error of P_C2
%               %obj.triangErrHistos{i}.addSig(P_C2_norm_err, zeros(size(P_C2_norm_err)));
%               obj.triangErrHistos{i}.update(P_C2_norm_err, zeros(size(P_C2_norm_err)));
%             end
%         end
%         
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         
%         % Procees final evaluation of all histograms
%         function evaluate(obj, P_N, R_C2C1, t_C2C1_C2)
%             % todo
%         end
%         
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         
%         function plot(obj)
%             %todo: plot histograms ...
%             for i = 1:1: length(obj.triangErrHistos)
%                 h = figure;
%                 figH = axis;
%                 plotH_v = [];
%                 plotStr_v = {};
%                 
%                 [plotH, plotStr] = obj.triangErrHistos{i}.plotOuterBins( figH );
%                 if ~isempty(plotH)
%                     plotH_v = [plotH, plotH_v];
%                     plotStr_v = {plotStr, plotStr_v{:}};
%                 end
%                 
%                 [plotH, plotStr] = obj.plotBins( figH );
%                 if ~isempty(plotH)
%                     plotH_v = [plotH, plotH_v];
%                     plotStr_v = {plotStr, plotStr_v{:}};
%                 end
%             end
%             
%         end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% function: update_container
        
        update_container(obj, camMotion, P_N1, P_N2, frameIdx, sequFrameNb)
        
        %% function: plot_numInvertedSignZ_oneframe
        plot_opticalFlow(obj, frameIdx, funcHandle_name, img )
        
        %% function: data_save 
        
        rawdata_save(obj, saveDir)


        %% function: triang_evaluation 
        %
        % the function for triangulation evaluation  
        %
        evaldata_multi_frame = triang_evaluation(obj, img,camMotion, intrinsicObj,P_N1, P_N2, gtPointCloudPlotFigHandle, variant,sequFrameNb, framesIdxToProcess,evaldata_multi_frame )
            
        %% function: triang_process
        
        triangdata = triang_process(obj, triangFuncHandles, triangFuncNames, variant);
        
        %% function:  plot_onePointMonteCarlo
        
        plot_onePointMonteCarlo(obj, frameIdx, pointIdx, funcHandle_name  )
        
        %% function: plot_triangdata
        
        [func_figHandle, compare_figHandle] = plot_triangdata(obj, variant)
        
        
        [mean_y, std_y, rsme_y, binPos_x, fig] = frequency_map(obj, xdata_origin, ydata_origin, rangeX, rangeY)
        
        %% plot_estStd(obj)
        
        plot_estStd(obj)
        
        
    end  % end of methods
    
    methods(Static)
        %% function: fig_save
        
        triangdata_save(triang_data, func_figHandle, compare_figHandle, funcHandle_name, variant)
        
        
        
        %% function: triang_process_MonteCarlo_oneframe
        
        triangdata = triang_process_MonteCarlo_oneframe(rawdata, frameIdx, flowSimObj, funcHandlesTriang)
        
        %% function: scatter_triangdata
        
        scatter_triangdata(triangdata_mat, funcHandleIdx, funcHandle_name, variant)
        
        
        
        %% function: plot_opticalFlow_numInvertedSignZ_oneframe
        plot_opticalFlow_numInvertedSignZ_oneframe(invertedsign_z_flag , gt_P_I1, gt_P_I2, funcHandleIdx, img_double, funcHandle_name, frameIdx)
        
        %% function: plot_opticalFlow_BiasOrStdOfErr_oneframe
        plot_opticalFlow_BiasOrStdOfErr_oneframe(error_ , gt_P_I1, gt_P_I2, foe_I, img_double, ...
            numFuncHandle, funcHandle_name, frameIdx, ObjParams, variant,invertedsign_z_flag)

        %%  plot_opticalFlow_parallaxDistribution
        plot_opticalFlow_parallaxDistribution(parallaxAngle , gt_P_I1, gt_P_I2, img_double, parallaxAngleThreshold)
    end
    
end