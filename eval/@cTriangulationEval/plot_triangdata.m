%% function: plot_triangdata
% author: <Lin, Wei> (CC-DA/ESV1)
% date: <2017-04-11>
% plot the frequency map and plots of comparison of bias/stddev
% INPUTS

function [func_figHandle, compare_figHandle] = plot_triangdata(obj, variant)

% description of error type, like 'Error of inv. z'
errType = obj.params.errorType;

switch errType
    case 'error radialdist'
        errTypeUnit = 'm';
        errTypeStructName = strrep(obj.params.errorType, ' ', '_');
        absMaxErr = obj.params.errorToleranceZ;
        
    case 'error invradialdist'
        errTypeUnit = 'm^{-1}';
        errTypeStructName = strrep(obj.params.errorType, ' ', '_');
        absMaxErr = obj.params.errorToleranceInvZ;
        
    case 'error rmse radialdist'
        errTypeUnit = 'm';
        errTypeStructName = strrep(obj.params.errorType, ' ', '_');
        absMaxErr = obj.params.errorToleranceZ;
        
    case 'error z'
        errTypeUnit = 'm';
        errTypeStructName = strrep(obj.params.errorType, ' ', '_');
        absMaxErr = obj.params.errorToleranceZ;
        
    case 'error invz'
        errTypeUnit = 'm^{-1}';
        errTypeStructName = strrep(obj.params.errorType, ' ', '_');
        absMaxErr = obj.params.errorToleranceInvZ;
        
    case 'error 3D'
        errTypeUnit = 'm';
        errTypeStructName = strrep(obj.params.errorType, ' ', '_');
        absMaxErr = obj.params.errorToleranceZ;
    case 'error 2D'
        errTypeUnit = 'pixel';
        errTypeStructName = strrep(obj.params.errorType, ' ', '_');
        absMaxErr = obj.params.errorToleranceZ;  
        
    otherwise
        error('Unknown error type!');
end

if obj.params.processingFlags.takeAbsoluteError
            absMinErr = 0;
else
    absMinErr = -absMaxErr;
end
plotLimits = [ ...
    obj.params.plot.freqMap.plotLimitsX, absMinErr, absMaxErr];

% the struct member name of error type in the struct triang_data, like 'error_invz'
%errTypeStructName = strrep(obj.params.errorType, ' ', '_');%      obj.params.errorTypeStructName;
% the unit of error type, like 'm^{-1}'
%errTypeUnit = obj.params.errorTypeUnit;

% description of x axis type, like 'parallax angle'
xAxisType = obj.params.xAxisType;
% the struct member name of x-axis type, like 'parallax_angle_gt' or 'parallax_angle_sim'
xAxisTypeStructName = [strrep(obj.params.xAxisType, ' ', '_'),'_',variant]; % obj.params.xAxisTypeStructName;
% the unit of x-aixs type, like 'deg'
xAxisUnit = obj.params.xAxisUnit;

%freqMapParams = obj.params.plot.freqMap;

% show the value parameters in the plot
switch variant
    case 'sim'
        str = {['std = (',num2str(obj.flowSimObj.params.std_I(1)),',',num2str(obj.flowSimObj.params.std_I(2)),') [pix]'],...
            [ 'tolerance max. z = ', num2str(obj.params.errorToleranceZ), ' [m]'],...
            ['tollerance min. z = ', num2str(obj.params.tooCloseToCameraToleranceZ), ' [m]'] };
        triangdata = obj.triang_data_sim;
    case 'gt'
        str = {'{std}=[0,0]',...
            [ 'tolerance max. z = ', num2str(obj.params.errorToleranceZ), ' [m]'],...
            ['tollerance min. z = ', num2str(obj.params.tooCloseToCameraToleranceZ), ' [m]'] };
       triangdata = obj.triang_data_gt;
end

num_funcHandles = size(triangdata.(errTypeStructName),1);
func_figHandle = cell(num_funcHandles,1);

errType_CompDataStruct = cell(num_funcHandles,1);

infoBoxPos = [0.14 .81 .1 .1];
numFrames = size(triangdata.(errTypeStructName), 3);

numIteration = size(triangdata.(errTypeStructName), 2);

% triangdata_mat.(xAxisTypeStructName) = cell2mat( obj.raw_data.(xAxisTypeStructName) );
% triangdata_mat.(xAxisTypeStructName) = triangdata_mat.(xAxisTypeStructName)(:)';

triangdata_mat.(xAxisTypeStructName) = cell2mat(reshape(obj.raw_data.(xAxisTypeStructName), 1, numIteration * numFrames));

% triangdata_mat.(xAxisTypeStructName) = cell2mat( triangdata.(xAxisTypeStructName)(1, :));


for funcHandleIdx = 1 : num_funcHandles
    
    triangdata_mat.(errTypeStructName) = cell2mat( triangdata.(errTypeStructName)(funcHandleIdx,:));
%     triangdata_mat.invertedsign_z_flag_mat = cell2mat( triangdata.invertedsign_z_flag(funcHandleIdx,:));
%     triangdata_mat.tooCloseToCamera_flag_mat = cell2mat( triangdata.tooCloseToCamera_flag(funcHandleIdx,:));
%     triangdata_mat.aboveErrorTolerance_flag_mat = cell2mat( triangdata.aboveErrorTolerance_flag(funcHandleIdx,:));
%     triangdata_mat.invertedsign_z_flag_mat = cell2mat( triangdata.invertedsign_z_flag(funcHandleIdx,:));
%     triangdata_mat.zC2Zero_flag_mat = cell2mat(triangdata.z_C2ZeroFlag(funcHandleIdx,:));
    
    % determine which data to consider in evaluation (removal of obvious
    % outliers)
%     dataToEvaluateFlags = ...
%         (~(triangdata_mat.tooCloseToCamera_flag_mat & obj.params.processingFlags.removeResultsWithinMinDistRange)) & ...           % too close points
%         (~(triangdata_mat.aboveErrorTolerance_flag_mat & obj.params.processingFlags.removeResultsAboveErrorTollerance)) & ...    % error above tollerance
%         (~(triangdata_mat.invertedsign_z_flag_mat & obj.params.processingFlags.removeInvertedResults));                            % inverted results
    
    %% frequency map without z in the range of ....
    
%     x_data = triangdata_mat.(xAxisTypeStructName)(dataToEvaluateFlags);
%     err_data = triangdata_mat.(errTypeStructName)(dataToEvaluateFlags);
    x_data = triangdata_mat.(xAxisTypeStructName);
    err_data = triangdata_mat.(errTypeStructName);
    
    if obj.params.processingFlags.takeAbsoluteError
        err_data = abs(err_data);
    end
    
%     numAboveTollerance = nnz(triangdata_mat.aboveErrorTolerance_flag_mat);
%     numValidDataPoints = nnz(isfinite(err_data));
%     numInverted = nnz(triangdata_mat.invertedsign_z_flag_mat);
%     numNonFinite = nnz(~isfinite(err_data));
%     numTooCloseToCamera = nnz(triangdata_mat.tooCloseToCamera_flag_mat);
%     numZero = nnz(triangdata_mat.zC2Zero_flag_mat);
    
    
    
    % plot the frequency map error of invz - parallax angle ( without points with z in range [-1 +1])
    [errType_CompDataStruct{funcHandleIdx,1}.bias, errType_CompDataStruct{funcHandleIdx,1}.stddev, errType_CompDataStruct{funcHandleIdx,1}.rsme, ...
     errType_CompDataStruct{funcHandleIdx,1}.bin_x, func_figHandle{funcHandleIdx,1}.(['fig_',errTypeStructName])] = ...
        obj.frequency_map(x_data, err_data, [obj.params.plot.freqMap.minX, obj.params.plot.freqMap.maxX], [-absMaxErr, absMaxErr]);
    
    axis(plotLimits)
    annotation('textbox',infoBoxPos,'String',str,'Interpreter','none');
    xlabel([xAxisType, ' [', xAxisUnit, ']']);
    ylabel([errType, ' [', errTypeUnit, ']']);
%     title(sprintf([errType, ' vs. ', xAxisType, ' for ',triangdata.funcNames{funcHandleIdx} ' (' variant ')\n', ...
%         'above tollerance ratio: ', num2str(numAboveTollerance/numValidDataPoints), ...
%         ', inverted sign: ', num2str(numInverted), ...
%         ', non finite: ', num2str(numNonFinite ), ...
%         ', too close: '   num2str(numTooCloseToCamera), ...
%         ', equal 0: ' num2str(numZero)]), ...
%         'Interpreter','none')
    

end  % end of for funcHandleIdx

%% plot of comparison of bias and stddev ( mean )
clr = jet(num_funcHandles);

% error of invz bias
compare_figHandle.([errTypeStructName, '_bias']) = figure('name',[errTypeStructName, 'bias_comp'], 'NumberTitle', 'off');

for funcHandleIdx = 1:1:num_funcHandles
    plot(errType_CompDataStruct{funcHandleIdx,1}.bin_x, errType_CompDataStruct{funcHandleIdx,1}.bias,'Color',clr(funcHandleIdx,:))
    hold on
end
annotation('textbox',infoBoxPos,'String',str,'Interpreter','none')
grid on
legend(triangdata.funcNames,'Interpreter','none')
xlabel([xAxisType, ' [', xAxisUnit, ']'])
ylabel([errType, ' [', errTypeUnit, ']'])
axis(plotLimits)
title(['Comparison ', errType, ' bias', ' (', variant, ')']);

% error of invz stddev
compare_figHandle.([errTypeStructName, '_stddev']) = figure('name',[errTypeStructName, 'stddev_comp'], 'NumberTitle', 'off');

for funcHandleIdx = 1:1:num_funcHandles
    plot(errType_CompDataStruct{funcHandleIdx,1}.bin_x, errType_CompDataStruct{funcHandleIdx,1}.stddev,'Color',clr(funcHandleIdx,:))
    hold on
end
annotation('textbox',infoBoxPos,'String',str,'Interpreter','none');
grid on
legend(triangdata.funcNames,'Interpreter','none');
xlabel([xAxisType, ' [', xAxisUnit, ']']);
ylabel([errType, ' [', errTypeUnit, ']']);
axis(plotLimits)
title(['Comparison ', errType, ' stddev', ' (', variant, ')']);

% error of invz rsme
compare_figHandle.([errTypeStructName, '_rsme']) = figure('name',[errTypeStructName, 'rsme_comp'], 'NumberTitle', 'off');

for funcHandleIdx = 1:1:num_funcHandles
    plot(errType_CompDataStruct{funcHandleIdx,1}.bin_x, errType_CompDataStruct{funcHandleIdx,1}.rsme,'Color',clr(funcHandleIdx,:))
    hold on
end
annotation('textbox',infoBoxPos,'String',str,'Interpreter','none');
grid on
legend(triangdata.funcNames,'Interpreter','none');
xlabel([xAxisType, ' [', xAxisUnit, ']']);
ylabel([errType, ' [', errTypeUnit, ']']);
axis(plotLimits)
title(['Comparison ', errType, ' rsme', ' (', variant, ')']);




%% Median/ Mean Error of iterations for each point
if strcmp(variant,'sim')
    
    averageStyle = obj.params.processingFlags.averageStyle;
    
    for funcHandleIdx = 1:num_funcHandles
        averageErr = cell(1, numFrames);
        
        switch averageStyle
            case 'Mean'
                for frameIdx = 1:numFrames
                        averageErr{1, frameIdx} = mean( cell2mat( triangdata.(errTypeStructName)(funcHandleIdx,:,frameIdx)' ));
                end
            case 'Median'
                for frameIdx = 1:numFrames
                    errSorted =  sort( cell2mat( triangdata.(errTypeStructName)(funcHandleIdx,:,frameIdx)' ));
                    averageErr{1, frameIdx} = median(errSorted);
                end
        end
        
        averageErr = cell2mat(averageErr);
        if obj.params.processingFlags.takeAbsoluteError
           averageErr = abs(averageErr); 
        end
        xData_ = cell2mat(obj.raw_data.parallax_angle_gt);
        % figure('name',[errTypeStructName, 'BiasCompWithAverageData'], 'NumberTitle', 'off')
        [errType_CompDataStruct{funcHandleIdx,1}.biasAverage, errType_CompDataStruct{funcHandleIdx,1}.stddevAverage, errType_CompDataStruct{funcHandleIdx,1}.rsmeAverage, ...
            errType_CompDataStruct{funcHandleIdx,1}.bin_xAverage, ~] = ...
            obj.frequency_map(xData_, averageErr, [obj.params.plot.freqMap.minX, obj.params.plot.freqMap.maxX], [-absMaxErr, absMaxErr]);
        
        axis(plotLimits)
        annotation('textbox',infoBoxPos,'String',str,'Interpreter','none');
        xlabel([xAxisType, ' [', xAxisUnit, ']']);
        ylabel([errType, ' [', errTypeUnit, ']']);
        title(sprintf([averageStyle, ' ',errType, ' vs. ', xAxisType, ' for ',triangdata.funcNames{funcHandleIdx} ' (' variant ')\n']), ...
            'Interpreter','none')
        
    end
    
    % comparison of bias    
    compare_figHandle.([errTypeStructName, '_bias', averageStyle]) = figure('name',[errTypeStructName, 'bias_comp',averageStyle], 'NumberTitle', 'off');
    
    for funcHandleIdx = 1:1:num_funcHandles
        plot(errType_CompDataStruct{funcHandleIdx,1}.bin_xAverage, errType_CompDataStruct{funcHandleIdx,1}.biasAverage,'Color',clr(funcHandleIdx,:))
        hold on
    end
    annotation('textbox',infoBoxPos,'String',str,'Interpreter','none')
    grid on
    legend(triangdata.funcNames,'Interpreter','none')
    xlabel([xAxisType, ' [', xAxisUnit, ']'])
    ylabel([averageStyle, ' ', errType, ' [', errTypeUnit, ']'])
    axis(plotLimits)
    title(['Comparison ', averageStyle,' ', errType, ' bias', ' (', variant, ')']);
    % comparison of std
    compare_figHandle.([errTypeStructName, '_stddev',averageStyle]) = figure('name',[errTypeStructName, 'stddev_comp',averageStyle], 'NumberTitle', 'off');
    
    for funcHandleIdx = 1:1:num_funcHandles
        plot(errType_CompDataStruct{funcHandleIdx,1}.bin_xAverage, errType_CompDataStruct{funcHandleIdx,1}.stddevAverage,'Color',clr(funcHandleIdx,:))
        hold on
    end
    annotation('textbox',infoBoxPos,'String',str,'Interpreter','none');
    grid on
    legend(triangdata.funcNames,'Interpreter','none');
    xlabel([xAxisType, ' [', xAxisUnit, ']']);
    ylabel([averageStyle, ' ',errType, ' [', errTypeUnit, ']']);
    axis(plotLimits)
    title(['Comparison ',averageStyle,' ', errType, ' stddev', ' (', variant, ')']);
    % comparison of rsme
    compare_figHandle.([errTypeStructName, '_rsme']) = figure('name',[errTypeStructName, 'rsme_comp', averageStyle], 'NumberTitle', 'off');
    
    for funcHandleIdx = 1:1:num_funcHandles
        plot(errType_CompDataStruct{funcHandleIdx,1}.bin_xAverage, errType_CompDataStruct{funcHandleIdx,1}.rsmeAverage,'Color',clr(funcHandleIdx,:))
        hold on
    end
    annotation('textbox',infoBoxPos,'String',str,'Interpreter','none');
    grid on
    legend(triangdata.funcNames,'Interpreter','none');
    xlabel([xAxisType, ' [', xAxisUnit, ']']);
    ylabel([errType, ' [', errTypeUnit, ']']);
    axis(plotLimits)
    title(['Comparison ',averageStyle,' ', errType, ' rsme', ' (', variant, ')']);
    
end

end