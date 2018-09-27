%% function: plot_frequency_map
% author: <Lin, Wei> (CC-DA/ESV1)
% date: <2017-03-31> 
%
% this function plots the frequency map with x axis representing input
% variable 1 and y axis representing input variable 2
% the color represents frequency of data in each small grid (by counting how many data fall into this grid)
% 
% INPUTS ( input parameters need to be further modified!! )
% xdata
% ydata
%                             
%
%
% OUTPUTS
% if the ydata_outliers_tolerance is given, compute bias, stddev and binC_x, 
% plot the frequency_map and return the figure handle ;
% if the ydata_outliers_tolerance is not given, the outliers will not be
% filtered out, then only bias, stddev and binC_x are computed but no plot
function [mean_y, std_y, rsme_y, binPos_x, fig] = frequency_map(obj, xdata_origin, ydata_origin, rangeX, rangeY)

freqMapParams = obj.params.plot.freqMap;

% filter out the NaNs and Infs
isfiniteFlags = isfinite(xdata_origin) & isfinite(ydata_origin);
xdata = xdata_origin(isfiniteFlags);
ydata = ydata_origin(isfiniteFlags);

% % the number of intervals along the x y axis
numBinX = freqMapParams.numBinX;
numBinY = freqMapParams.numBinY;


x_edges = linspace(rangeX(1), rangeX(2), numBinX);
y_edges = linspace(rangeY(1), rangeY(2), numBinY);
%binPos_x = x_edges(1:end-1) + diff(x_edges)/2;
%binPos_y = y_edges(1:end-1) + diff(y_edges)/2;

[frequency_map_, binPos_x, binPos_y] = histcounts2(xdata, ydata, x_edges, y_edges);

frequency_map_log = (log10(frequency_map_))';
%frequency_map_log(frequency_map_log==-Inf) = NaN;

% limit the y range (with minY and maxY) for better visualization 
% YRange_logic = (binC_y >= freqMapParams.minY & binC_y <= freqMapParams.maxY);
% binC_y = binC_y(YRange_logic);
% frequency_map_log = frequency_map_log(YRange_logic, :);

% the precise bias and stddev
dataBinsX = discretize(xdata, x_edges);
mean_y = zeros(1, numBinX);
std_y = zeros(1, numBinX);
rsme_y = zeros(1, numBinX);
for binX_idx = 1 : numBinX
    mean_y(binX_idx) = mean(ydata(1,dataBinsX == binX_idx));
    std_y(binX_idx) = std(ydata(1,dataBinsX == binX_idx));
    rsme_y(binX_idx) = sqrt(mean(ydata(1,dataBinsX == binX_idx)).^2);
end

% the approximate bias and stddev (almost the same but faster)
% bias_mean_ = (binC_y * frequency_map_') ./ sum(frequency_map_',1);
% std_dev_ = sqrt(sum((((repmat(binC_y',1,length(bias_mean)) - repmat(bias_mean,length(binC_y),1)).^2).*frequency_map_'),1) ./ sum(frequency_map_',1));


avg_bias = mean(ydata);
avg_stddev = std(ydata);
%avg_bias_with_outliers = mean(ydata_finite);
%avg_stddev_with_outliers = std(ydata_finite);

fig = figure('name','frequency map', 'NumberTitle', 'off');
imagesc_h = imagesc([binPos_x(1),binPos_x(end)], [binPos_y(1),binPos_y(end)], frequency_map_log);
%set(imagesc_h, 'AlphaData',frequency_map_log==-Inf) % log10(0) = -inf, only show the finite data
cmap = flipud(parula(1024));
colormap(cmap)
set(gca,'YDir','normal')
cb = colorbar;
cb.Label.String = 'log_{10} frequency';
grid on
hold on
biasH = plot(binPos_x, mean_y,'Color','g','Linewidth',1.5)             % bias
stdH = plot(binPos_x, mean_y + std_y,'Color','r','Linewidth',1.5)   % std
if ~obj.params.processingFlags.takeAbsoluteError
    plot(binPos_x, mean_y - std_y,'Color','r','Linewidth',1.5)   % std
end
avgBiasH = plot(binPos_x, avg_bias*ones(size(binPos_x)),'g--')                 % avg. bias
avgStdH = plot(binPos_x, (avg_bias+avg_stddev)*ones(size(binPos_x)),'r--')    % avg. std
plot(binPos_x, (avg_bias-avg_stddev)*ones(size(binPos_x)),'r--')    % avg. std

% plot(binC_x, avg_bias_with_outliers*ones(size(binC_x)),'b--')
% plot(binC_x, (avg_bias_with_outliers+avg_stddev_with_outliers)*ones(size(binC_x)),'m--')
%plot(binC_x, (avg_bias_with_outliers-avg_stddev_with_outliers)*ones(size(binC_x)),'m--')

legend([biasH, stdH, avgBiasH, avgStdH], ...
    'bias', 'std', ['avg. bias ', num2str(avg_bias)], ['avg.stddev ', num2str(avg_stddev)]);

end