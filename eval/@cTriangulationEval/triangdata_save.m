%% function: triangdata_save
% author: <Lin, Wei> (CC-DA/ESV1)
% date: <2017-04-11>
% save the triang data, frequency maps and plots of comparison of bias/stddev

% INPUTS
function triangdata_save(triang_data, func_figHandle, compare_figHandle, funcHandle_name, variant)

currFilePath = mfilename('fullpath');

if ~exist( fullfile(currFilePath(1:end-15), 'TriangEvalObj_triangdata'),'dir')
    mkdir( fullfile(currFilePath(1:end-15), 'TriangEvalObj_triangdata'))
end

folderFullName = fullfile(currFilePath(1:end-15), 'TriangEvalObj_triangdata', [variant, 'TriangData_', getDateString()]);
% create a new folder
mkdir(fullfile(folderFullName, 'frequency_map'));
mkdir(fullfile(folderFullName, 'comparison_plots'));
dataName = 'Triangdata';
save(fullfile(folderFullName, [variant,dataName, '_', getDateString(), '.mat']), 'triang_data' )

%% save plots of frequency map
frquency_map_folder = fullfile(folderFullName, 'frequency_map');
comparison_plots_folder = fullfile(folderFullName, 'comparison_plots');

for i = 1:1:size(func_figHandle,1)
    saveas(func_figHandle{i,1}.fig_error_invz, fullfile(frquency_map_folder, [variant,funcHandle_name{1,i},'_errinvz.fig']))
    saveas(func_figHandle{i,1}.fig_error_z, fullfile(frquency_map_folder, [variant,funcHandle_name{1,i},'_errz.fig']))
    saveas(func_figHandle{i,1}.fig_error_radialdist, fullfile(frquency_map_folder, [variant,funcHandle_name{1,i},'_errradialdist.fig']))
    saveas(func_figHandle{i,1}.fig_error_invz_without, fullfile(frquency_map_folder, [variant,funcHandle_name{1,i},'_errinvz_wo.fig']))
    saveas(func_figHandle{i,1}.fig_error_z_without, fullfile(frquency_map_folder, [variant,funcHandle_name{1,i},'_errz_wo.fig']))
    saveas(func_figHandle{i,1}.fig_error_radialdist_without, fullfile(frquency_map_folder, [variant,funcHandle_name{1,i},'_errradialdist_wo.fig']))
end

%% save plots of comparison of bias/stddev
saveas(compare_figHandle.errinvz_bias, fullfile(comparison_plots_folder, [variant,'errinvz_bias.fig']))
saveas(compare_figHandle.errinvz_stddev, fullfile(comparison_plots_folder, [variant,'errinvz_stddev.fig']))
saveas(compare_figHandle.errz_bias, fullfile(comparison_plots_folder, [variant,'errz_bias.fig']))
saveas(compare_figHandle.errz_stddev, fullfile(comparison_plots_folder, [variant,'errz_stddev.fig']))
saveas(compare_figHandle.errradialdist_bias, fullfile(comparison_plots_folder, [variant,'errradialdist_bias.fig']))
saveas(compare_figHandle.errradialdist_stddev, fullfile(comparison_plots_folder, [variant,'errradialdist_stddev.fig']))

saveas(compare_figHandle.errinvz_bias_with_outlier, fullfile(comparison_plots_folder, [variant,'errinvz_bias_with_outlier.fig']))
saveas(compare_figHandle.errinvz_stddev_with_outlier, fullfile(comparison_plots_folder, [variant,'errinvz_stddev_with_outlier.fig']))
saveas(compare_figHandle.errz_bias_with_outlier, fullfile(comparison_plots_folder, [variant,'errz_bias_with_outlier.fig']))
saveas(compare_figHandle.errz_stddev_with_outlier, fullfile(comparison_plots_folder, [variant,'errz_stddev_with_outlier.fig']))
saveas(compare_figHandle.errradialdist_bias_with_outlier, fullfile(comparison_plots_folder, [variant,'errradialdist_bias_with_outlier.fig']))
saveas(compare_figHandle.errradialdist_stddev_with_outlier, fullfile(comparison_plots_folder, [variant,'errradialdist_stddev_with_outlier.fig']))


disp(['Triang data saved at: ',folderFullName] )

end

