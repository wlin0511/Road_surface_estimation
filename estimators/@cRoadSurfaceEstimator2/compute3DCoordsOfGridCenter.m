function meshP_C = compute3DCoordsOfGridCenter(obj, intrinsicObj, gridData, variant )

% if ndims(gridData) == 2
%     gridData = reshape(gridData, size(gridData,1), size(gridData,2), 1);
% end
numTimeLags = size(gridData,3);

% P_N = intrinsicObj.invProjPinHole( imgCoordsData) ;
[imgCoordsXGridCenter, imgCoordsYGridCenter] = obj.grid.getImgCoordsOfGridCenter();

[meshImgCoordsX, meshImgCoordsY] = meshgrid( imgCoordsXGridCenter, imgCoordsYGridCenter);
imgCoordsGridCenter = [meshImgCoordsX(:)'; meshImgCoordsY(:)'];
P_N_imgCoordsGridCenter = intrinsicObj.invProjPinHole( imgCoordsGridCenter);

meshP_C = zeros(size(gridData,1), size(gridData,2), 3, numTimeLags);
for timeLagIdx=1:numTimeLags
    gridData_ = gridData(:,:,timeLagIdx);
    switch variant
        case 'invZ'
            P_C = (1./gridData_(:)') .* P_N_imgCoordsGridCenter;
        case 'invRadialDist'
            P_C = (1./gridData_(:)') .* vec2unitVec(P_N_imgCoordsGridCenter);
        case 'Z'
            P_C = (gridData_(:)') .* P_N_imgCoordsGridCenter;
    end

    for i=1:3
        meshP_C(:,:,i,timeLagIdx ) = reshape(P_C(i,:),size(gridData,1), size(gridData,2) );
    end
end
    

    



end