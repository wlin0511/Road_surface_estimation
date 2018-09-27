function plotHandle = meshGridData(obj, gridData, color, variant)

if variant == 'Z'
   variantUnit = '[m]'; 
else
    variantUnit ='[m]';
end
imageHorCoords = obj.grid.cellIdxHorToCoordinates(1: size(gridData, 2));
imageVerCoords = obj.grid.cellIdxVerToCoordinates(1: size(gridData, 1));

if isscalar(imageHorCoords)
    % plot(imageVerCoords, gridData',color)
    scatterHandle = scatter(imageVerCoords, gridData',color,'.');
    if nargout>0
        plotHandle = scatterHandle;
    end
    xlabel('Y[pixel]')
    ylabel([variant,variantUnit])
else
    [meshX, meshY] = meshgrid(imageHorCoords, imageVerCoords );
    mesh(meshX,meshY, gridData, 'EdgeColor',color)
    xlabel('X[pixel]')
    ylabel('Y[pixel]')
    zlabel([variant, variantUnit])
end