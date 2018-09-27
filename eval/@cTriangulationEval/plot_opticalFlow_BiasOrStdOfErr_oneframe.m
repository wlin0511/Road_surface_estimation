function plot_opticalFlow_BiasOrStdOfErr_oneframe(error_ , gt_P_I1, gt_P_I2, foe_I, img_double, ...
    numFuncHandle, funcHandle_name, frameIdx, ObjParams, variant,invertedsign_z_flag)

errTypeStructName = strrep(ObjParams.errorType, ' ', '_');

% filter out the outlier in order to have a better visualization
switch variant
    case 'bias'
        outlier_tolerance = ObjParams.plot.opticalFlow.biasTolerance;
    case 'std'
        outlier_tolerance = ObjParams.plot.opticalFlow.stdTolerance;
end

biasOrStdOfErr = cell(numFuncHandle,1);
biasOrStdOfErr_inlier = cell(numFuncHandle,1);
for funcHandleIdx = 1:1:numFuncHandle
    error_mat = cell2mat(error_(funcHandleIdx, :, frameIdx)');
    if nargin > 10      
        invertedsign_z_flag_mat = double(~cell2mat(invertedsign_z_flag(funcHandleIdx, :)')); % 0 stands for inverted sign   
        invertedsign_z_flag_mat ( invertedsign_z_flag_mat == 0) = NaN;
        error_mat = error_mat.*invertedsign_z_flag_mat; % filter out the points with inverted sign of z by setting them as NaN
    end
    switch variant
        case 'bias'
            biasOrStdOfErr{funcHandleIdx,1} = median(error_mat,'omitnan');
        case 'std'
            biasOrStdOfErr{funcHandleIdx,1} = std(error_mat,'omitnan');
    end
    biasOrStdOfErr_inlier{funcHandleIdx,1} = biasOrStdOfErr{funcHandleIdx,1};
    biasOrStdOfErr_inlier{funcHandleIdx,1}( abs(biasOrStdOfErr_inlier{funcHandleIdx,1}) > outlier_tolerance ) = NaN;
end

% min and max values for setting the limits of the common colorbar
bottom_ = min(min(cell2mat(biasOrStdOfErr_inlier)));
top_ = max(max(cell2mat(biasOrStdOfErr_inlier)));


for funcHandleIdx = 1:1:numFuncHandle
    figure('name',[variant,'Of',errTypeStructName,' ',funcHandle_name{funcHandleIdx}],'NumberTitle', 'off');
    imshow(repmat(img_double,1,1,3))
    hold on
    scatter(foe_I(1),foe_I(2),'filled','r')
    plotColorLines( gt_P_I1, gt_P_I2, biasOrStdOfErr_inlier{funcHandleIdx,1})  
    %  > outlier_tolerance  :  red
    plotLines(gt_P_I1(:,( biasOrStdOfErr{funcHandleIdx,1} > outlier_tolerance )),gt_P_I2(:,( biasOrStdOfErr{funcHandleIdx,1} > outlier_tolerance )),'r');
    % < -outlier_tolerance : magenta
    plotLines(gt_P_I1(:,( biasOrStdOfErr{funcHandleIdx,1} < -outlier_tolerance )),gt_P_I2(:,( biasOrStdOfErr{funcHandleIdx,1} < -outlier_tolerance )),'m');
    % set the limmits of the colorbar to manual
    caxis manual
    caxis([bottom_ top_]);
    cb = colorbar;
    cb.Label.String = [variant, ' of error'];
    if nargin > 10
        title({[funcHandle_name{funcHandleIdx}, ' ',variant,' of ',ObjParams.errorType,' (without inverted sign of z) in all iterations in frame ', num2str(frameIdx)];...
            ['outlier tolerance ', num2str(outlier_tolerance), '  >outliers:red']},'Interpreter','none')
    else
        title({[funcHandle_name{funcHandleIdx}, ' ', variant,' of ',ObjParams.errorType,' in all iterations in frame ', num2str(frameIdx)];...
            ['outlier tolerance ', num2str(outlier_tolerance), '  >outliers:red']},'Interpreter','none')
    end
end


end