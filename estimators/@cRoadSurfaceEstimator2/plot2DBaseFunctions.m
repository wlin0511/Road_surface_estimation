
function plot2DBaseFunctions(obj, baseFuncHor,baseFuncVer, A  )

baseFuncImageHorCoords = obj.grid.cellIdxHorToCoordinates(1: size(baseFuncHor, 1));
baseFuncImageVerCoords = obj.grid.cellIdxVerToCoordinates(1: size(baseFuncVer, 1));
if ~isscalar(baseFuncImageHorCoords)
    [meshXbaseFuncHor, meshYbaseFuncVer] = meshgrid(baseFuncImageHorCoords, baseFuncImageVerCoords);
end

for i = 1:size(baseFuncHor, 2)
    for j = 1:size(baseFuncVer, 2)
        baseFunc2D = baseFuncVer(:,j) * A(j,i) * baseFuncHor(:,i)';
        % baseFunc2D(baseFunc2D == 0) =NaN;
        if isscalar(baseFuncImageHorCoords)
            plot(baseFuncImageVerCoords, baseFunc2D')
        else
            mesh( meshXbaseFuncHor,meshYbaseFuncVer, baseFunc2D, 'AlphaData',0.5 )
        end
        hold on
    end
end



end
% plot the 2D base function
