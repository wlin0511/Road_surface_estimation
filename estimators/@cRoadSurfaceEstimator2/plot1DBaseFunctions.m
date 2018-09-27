function plot1DBaseFunctions(obj, baseFuncVer, A  )

baseFuncImageVerCoords = obj.grid.cellIdxVerToCoordinates(1: size(baseFuncVer, 1));

for j = 1:size(baseFuncVer,2)
   plot(baseFuncImageVerCoords, baseFuncVer(:,j)' * A(j)) 
   hold on
end

end