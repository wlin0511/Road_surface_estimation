function plot_opticalFlow(obj, frameIdx, funcHandle_name, img )

% description of error type, like 'error invz'
% errType = ObjParams.errorType; 
% the struct member name of error type in the struct triang_data, like 'error_invz'
errTypeStructName = strrep(obj.params.errorType, ' ', '_');
% the unit of error type, like 'm^{-1}'
% errTypeUnit = ObjParams.errorTypeUnit;

numFuncHandle = length(funcHandle_name);
% numIteration = size(obj.triang_data_sim.invertedsign_z_flag,2);

gt_P_N1 = obj.raw_data.gt_P_N1{1,frameIdx};
gt_P_N2 = obj.raw_data.gt_P_N2{1,frameIdx};
gt_P_I1 = obj.flowSimObj.simIntrinsics.projPinHoleN(gt_P_N1);
gt_P_I2 = obj.flowSimObj.simIntrinsics.projPinHoleN(gt_P_N2);

img_double = im2double(img)/max(max(im2double(img)));

foe_I = obj.flowSimObj.simIntrinsics.projPinHoleN(obj.raw_data.camMotion_gt{1,1}.t_C2C1_C2);

% figure,
% imshow(repmat(img_double,1,1,3))
% hold on
% scatter(foe_I(1),foe_I(2),'filled','w')
% title('parallax angle distribution in optical flow field','Interpreter','none')
% set(gcf,'color','w');


figure('name', 'parallaxAngleDistribution', 'NumberTitle', 'off')
cTriangulationEval.plot_opticalFlow_parallaxDistribution(obj.raw_data.parallax_angle_gt{1, frameIdx} , gt_P_I1, gt_P_I2, img_double, 0.5)
scatter(foe_I(1),foe_I(2),'filled','w')

for funcHandleIdx = 1:1:numFuncHandle
    % frequency of iverted sign of z among all iterations in one frame
    figure('name',['numInvertedSignZ ',funcHandle_name{funcHandleIdx}], 'NumberTitle', 'off');
    cTriangulationEval.plot_opticalFlow_numInvertedSignZ_oneframe(obj.triang_data_sim.invertedsign_z_flag , gt_P_I1, gt_P_I2, funcHandleIdx, img_double, funcHandle_name, frameIdx)
    
end

invertedsign_z_flag = obj.triang_data_sim.invertedsign_z_flag(:, :, frameIdx);


% bias of error type
cTriangulationEval.plot_opticalFlow_BiasOrStdOfErr_oneframe(obj.triang_data_sim.(errTypeStructName) , ...
    gt_P_I1, gt_P_I2, foe_I, img_double, numFuncHandle, funcHandle_name, frameIdx, obj.params, 'bias')
cTriangulationEval.plot_opticalFlow_BiasOrStdOfErr_oneframe(obj.triang_data_sim.(errTypeStructName) , ...
    gt_P_I1, gt_P_I2, foe_I, img_double, numFuncHandle, funcHandle_name, frameIdx, obj.params, 'bias', invertedsign_z_flag)
% std of error type
cTriangulationEval.plot_opticalFlow_BiasOrStdOfErr_oneframe(obj.triang_data_sim.(errTypeStructName) , ...
    gt_P_I1, gt_P_I2, foe_I, img_double, numFuncHandle, funcHandle_name, frameIdx, obj.params,'std')
cTriangulationEval.plot_opticalFlow_BiasOrStdOfErr_oneframe(obj.triang_data_sim.(errTypeStructName) , ...
    gt_P_I1, gt_P_I2, foe_I, img_double, numFuncHandle, funcHandle_name, frameIdx, obj.params,'std', invertedsign_z_flag)


end