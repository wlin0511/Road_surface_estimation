
% this plot function is for the 'sim' case
function plot_estStd(obj)


% errType = obj.params.errorType;
errType = 'error radialdist';
errTypeStructName = 'error_radialdist';
% switch errType
%     case 'error radialdist'
%         errTypeUnit = 'm';
%         errTypeStructName = strrep(obj.params.errorType, ' ', '_');
%         plotLimits = [obj.params.plot.freqMap.plotLimitsX, obj.params.plot.freqMap.plotLimitsY];
%         absMaxErr = obj.params.errorToleranceZ;
%         
%     case 'error invradialdist'
%         errTypeUnit = 'm^{-1}';
%         errTypeStructName = strrep(obj.params.errorType, ' ', '_');
%         plotLimits = [ ...
%             obj.params.plot.freqMap.plotLimitsX, ...
%             ones(1,2) ./ obj.params.plot.freqMap.plotLimitsY];
%         absMaxErr = obj.params.errorToleranceInvZ;
%         
%     case 'error rmse radialdist'
%         errTypeUnit = 'm';
%         errTypeStructName = strrep(obj.params.errorType, ' ', '_');
%         plotLimits = [ ...
%             obj.params.plot.freqMap.plotLimitsX, obj.params.plot.freqMap.plotLimitsY];
%         absMaxErr = obj.params.errorToleranceZ;
%         
%     case 'error z'
%         errTypeUnit = 'm';
%         errTypeStructName = strrep(obj.params.errorType, ' ', '_');
%         plotLimits = [obj.params.plot.freqMap.plotLimitsX, obj.params.plot.freqMap.plotLimitsY];
%         absMaxErr = obj.params.errorToleranceZ;
%         
%     case 'error invz'
%         errTypeUnit = 'm^{-1}';
%         errTypeStructName = strrep(obj.params.errorType, ' ', '_');
%         plotLimits = [ ...
%             obj.params.plot.freqMap.plotLimitsX, ...
%             ones(1,2) ./ obj.params.plot.freqMap.plotLimitsY];
%         absMaxErr = obj.params.errorToleranceInvZ;
%         
%     otherwise
%         error('Unknown error type!');
% end

xAxisType = obj.params.xAxisType;
xAxisTypeStructName = [strrep(obj.params.xAxisType, ' ', '_'),'_','sim'];
xAxisUnit = obj.params.xAxisUnit;


% str = {['std = (',num2str(obj.flowSimObj.params.std_I(1)),',',num2str(obj.flowSimObj.params.std_I(2)),') [pix]'],...
%     [ 'tolerance max. z = ', num2str(obj.params.errorToleranceZ), ' [m]'],...
%     ['tollerance min. z = ', num2str(obj.params.tooCloseToCamera_tolerance), ' [m]'] };
triangdata = obj.triang_data_sim;

num_funcHandles = size(triangdata.(errTypeStructName),1);

infoBoxPos = [0.14 .81 .1 .1];

numFrames = size(triangdata.(errTypeStructName), 3);

numIteration = size(triangdata.(errTypeStructName), 2);


% triangdata_mat.(xAxisTypeStructName) = cell2mat(reshape(obj.raw_data.(xAxisTypeStructName), 1, numIteration * numFrames));
triangdata_mat.(xAxisTypeStructName) = cell2mat(obj.raw_data.(xAxisTypeStructName));


for funcHandleIdx = 1 : num_funcHandles
    triangdata_mat.(errTypeStructName) = cell(1, numFrames);
    triangdata_mat.est_Std = cell(1, numFrames);
    
    for frameIdx = 1:numFrames
        
        triangdata_mat.(errTypeStructName){1, frameIdx} = cell2mat( reshape( triangdata.(errTypeStructName)(funcHandleIdx,:,frameIdx ), numIteration, 1)) ;
        
        triangdata_mat.est_Std {1, frameIdx} = cell2mat( reshape( triangdata.est_Std(funcHandleIdx,:,frameIdx ), numIteration, 1)) ;
    end
    
    triangdata_mat.(errTypeStructName) = cell2mat(triangdata_mat.(errTypeStructName));
    
    triangdata_mat.est_Std = cell2mat(triangdata_mat.est_Std);
    
    % sort the parallax angle and the err data accordingly
    [~, xAxisTypeOrder] = sort(triangdata_mat.(xAxisTypeStructName)(1,:));
    triangdata_mat.(xAxisTypeStructName) = triangdata_mat.(xAxisTypeStructName)(:, xAxisTypeOrder);
    triangdata_mat.(errTypeStructName) = triangdata_mat.(errTypeStructName)(:, xAxisTypeOrder);
    triangdata_mat.est_Std = triangdata_mat.est_Std(:, xAxisTypeOrder); 
    
%     figure, scatter( 1:length(triangdata_mat.(xAxisTypeStructName)), triangdata_mat.(xAxisTypeStructName),'.')
%     xlabel('data index')
%     ylabel(xAxisType)

    uniqueParallax = unique(triangdata_mat.(xAxisTypeStructName)(1,:));

    parallaxChosen = uniqueParallax(1:50:end);
    
    figure,
    for i = 1:length(parallaxChosen)
        parallaxIdx = find(triangdata_mat.(xAxisTypeStructName)(1,:)==parallaxChosen(i));
        x_ = triangdata_mat.(xAxisTypeStructName)(:,parallaxIdx);
        y_ = triangdata_mat.est_Std(:,parallaxIdx);
        z_ = triangdata_mat.(errTypeStructName)(:,parallaxIdx);
        scatter3(x_(:),y_(:),abs(z_(:)),'.')
        hold on
    end
    xlabel(xAxisType)
    ylabel('est std')
    zlabel(errType)
%     savefig('parallax-estStd-errRadialDist.fig')
    
end




%     triangdata_mat.invertedsign_z_flag_mat = cell2mat( triangdata.invertedsign_z_flag(funcHandleIdx,:));
%     triangdata_mat.tooCloseToCamera_flag_mat = cell2mat( triangdata.tooCloseToCamera_flag(funcHandleIdx,:));
%     triangdata_mat.aboveErrorTolerance_flag_mat = cell2mat( triangdata.aboveErrorTolerance_flag(funcHandleIdx,:));
%     triangdata_mat.invertedsign_z_flag_mat = cell2mat( triangdata.invertedsign_z_flag(funcHandleIdx,:));
%     triangdata_mat.zC2Zero_flag_mat = cell2mat(triangdata.z_C2ZeroFlag(funcHandleIdx,:));
%     
%     % determine which data to consider in evaluation (removal of obvious
%     % outliers)
%     dataToEvaluateFlags = ...
%         (~(triangdata_mat.tooCloseToCamera_flag_mat & obj.params.processingFlags.removeResultsWithinMinDistRange)) & ...           % too close points
%         (~(triangdata_mat.aboveErrorTolerance_flag_mat & obj.params.processingFlags.removeResultsAboveErrorTollerance)) & ...    % error above tollerance
%         (~(triangdata_mat.invertedsign_z_flag_mat & obj.params.processingFlags.removeInvertedResults));                            % inverted results
% 
%     if obj.params.processingFlags.takeAbsoluteError
%         err_data = abs(err_data);
%     end
% 
% 
%     numAboveTollerance = nnz(triangdata_mat.aboveErrorTolerance_flag_mat);
%     numValidDataPoints = nnz(isfinite(err_data));
%     numInverted = nnz(triangdata_mat.invertedsign_z_flag_mat);
%     numNonFinite = nnz(~isfinite(err_data));
%     numTooCloseToCamera = nnz(triangdata_mat.tooCloseToCamera_flag_mat);
%     numZero = nnz(triangdata_mat.zC2Zero_flag_mat);
% 
% 
%     axis(plotLimits)
%     annotation('textbox',infoBoxPos,'String',str,'Interpreter','none');
%     xlabel([xAxisType, ' [', xAxisUnit, ']']);
%     ylabel([errType, ' [', errTypeUnit, ']']);
%     title(sprintf([errType, ' vs. ', xAxisType, ' for ',triangdata.funcNames{funcHandleIdx} ' (' variant ')\n', ...
%         'above tollerance ratio: ', num2str(numAboveTollerance/numValidDataPoints), ...
%         ', inverted sign: ', num2str(numInverted), ...
%         ', non finite: ', num2str(numNonFinite ), ...
%         ', too close: '   num2str(numTooCloseToCamera), ...
%         ', equal 0: ' num2str(numZero)]), ...
%         'Interpreter','none')













end