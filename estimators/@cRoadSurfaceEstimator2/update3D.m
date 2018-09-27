function [ gridDataBeforeSmoo, gridWeight] = update3D(obj, P_C2_S_cell, intrinsicObj, variant, pointsStd_cell)
disp('Road surface: update 3D');
obj.grid.clear();

numPointClouds = length(P_C2_S_cell);

gridDataBeforeSmoo = NaN(obj.grid.h, obj.grid.w, numPointClouds);
gridWeight = NaN(obj.grid.h, obj.grid.w, numPointClouds);

for i = 1:numPointClouds
    P_C2_S = P_C2_S_cell{1,i};
    assert(~isempty(P_C2_S), 'Empty point cloud!' )
    obj.grid.clear();
    % project the points to get image coordinates
    indexDataGrid = intrinsicObj.projPinHole(P_C2_S); % P_I2 = (X, Y)
    
    numPoints = size( P_C2_S, 2);
    if nargin < 5
        pointsStd = 1 * ones (1, numPoints); % default setting: Std = 1
    else
        pointsStd = pointsStd_cell{1,i};
    end
    
    switch variant
        case 'invZ'
            Data = ones(1, numPoints) ./ P_C2_S(3,:);
        case 'invRadialDist'
            Data = ones(1, numPoints) ./column_norm(P_C2_S);
        case 'Z'
            Data = P_C2_S(3,:);
    end
    % put the data and weights in the DataGrid and StdGrid
    obj.grid.addCorrespondences(indexDataGrid, Data, pointsStd);
    
    [gridDataBeforeSmoo(:,:,i), gridStd] = obj.grid.cellModesWeighted();
    assert(any(any(gridStd==0))==0, 'Std of value zero in gridStd!!') % throw error if there are zero values in gridStd
    gridWeight_ = 1./gridStd;
    % set the weights of grids which have no data as 0
    gridWeight_ (isnan(gridWeight_)) = 0;
    gridWeight(:,:,i) = gridWeight_;
    
    
end


end