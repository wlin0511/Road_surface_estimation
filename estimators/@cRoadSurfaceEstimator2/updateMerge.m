function [ gridDataBeforeSmoo, gridWeight] = updateMerge(obj, P_C2_S_cell, intrinsicObj, variant, pointsStd_cell)

disp('Road surface: update fusion');
obj.grid.clear();

% merge the pointclouds into one pointcloud
P_C2_S = cell2mat(P_C2_S_cell);

assert(~isempty(P_C2_S), 'Empty point cloud!' )

indexDataGrid = intrinsicObj.projPinHole(P_C2_S);  % P_I2 = (X, Y)
numPoints = size( P_C2_S, 2);

if nargin < 5
    pointsStd = 1 * ones (1, numPoints); % default setting: Std = 1
else
    % merge the std data into one pointcloud
    pointsStd = cell2mat(pointsStd_cell);
end

switch variant
    case 'invZ'
        Data = ones(1, numPoints) ./ P_C2_S(3,:);
    case 'invRadialDist'
        Data = ones(1, numPoints) ./column_norm(P_C2_S);
end

% put the data and weights in the DataGrid and StdGrid
obj.grid.addCorrespondences(indexDataGrid, Data, pointsStd);
    
[gridDataBeforeSmoo, gridStd] = obj.grid.cellModesWeighted();
assert(any(any(gridStd==0))==0, 'Std of value zero in gridStd!!') % throw error if there are zero values in gridStd
gridWeight = 1./gridStd;
% set the weights of grids which have no data as 0
gridWeight (isnan(gridWeight)) = 0;

end