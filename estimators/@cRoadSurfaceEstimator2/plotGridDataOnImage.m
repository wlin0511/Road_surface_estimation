function plotGridDataOnImage(obj, img, DataGrid, knotsImgHorCoords, knotsImgVerCoords, baseFuncHor, baseFuncVer, cmapLimits, variant )

if strcmp(variant, 'Z')||strcmp(variant, 'Y')
   variantUnit = '[m]'; 
elseif strcmp(variant, 'Grid Weight')
    variantUnit = []; 
else
    variantUnit ='[m^(-1)]';
end

imshow(repmat(uint8(img*255),1,1,3));
hold on
% obj.grid.plot('b') % plot horizontal/vertical grid lines
% hold on
% plot knots over the image
[meshgridX, meshgridY] = meshgrid(knotsImgHorCoords, knotsImgVerCoords);
scatter(meshgridX(:), meshgridY(:), 'filled', 'k')

if ( ~isempty(baseFuncHor))|| (~isempty(baseFuncVer))
    baseFuncImageHorCoords = obj.grid.cellIdxHorToCoordinates(1: size(baseFuncHor, 1));
    baseFuncImageVerCoords = obj.grid.cellIdxVerToCoordinates(1: size(baseFuncVer, 1));
    
    % plot the 1D base functions in horizontal/vertical direction
    for i = 1:size(baseFuncHor, 2)
        plot(baseFuncImageHorCoords, -100*baseFuncHor(:,i))
    end
    
    for j = 1:size(baseFuncVer, 2)
        plot(-100*baseFuncVer(:,j), baseFuncImageVerCoords )
    end
    
end
xlabel('x [pixel]');
ylabel('y [pixel]');

x_edges = linspace(obj.grid.horLimits(1), obj.grid.horLimits(2), size(DataGrid, 2)+1);
y_edges = linspace(obj.grid.verLimits(1), obj.grid.verLimits(2), size(DataGrid, 1)+1);
binC_x = x_edges(1:end-1)+diff(x_edges)/2;
binC_y = y_edges(1:end-1)+diff(y_edges)/2;
hold on
if isscalar(binC_x)
    imagesc_h = imagesc( [(1+binC_x)/2, (size(img,2)+binC_x)/2],[binC_y(1),binC_y(end)],[DataGrid, DataGrid]);
    set(imagesc_h,'AlphaData', 0.5*isfinite([DataGrid, DataGrid])) 
else  % 2D case
    % logDataGrid = log(DataGrid);
    imagesc_h = imagesc( [binC_x(1),binC_x(end)],[binC_y(1),binC_y(end)],DataGrid );
    set(imagesc_h,'AlphaData', 0.5*isfinite(DataGrid)) 
end

if strcmp(variant, 'Grid Weight')||strcmp(variant, 'Y')
    cmap = hsv;
else
    
    cmap = flipud(jet);
%     cmap =hsv;
    %cmap = [darkHSV(1024, 4,4);darkHSV(1024, 4,3);darkHSV(1024, 4,2);darkHSV(1024, 4,1)];
end

colormap(cmap)

caxis manual
caxis([cmapLimits(1) cmapLimits(2)]);
cb = colorbar;
cb.Label.String = [variant, variantUnit];

axis on
ax = gca; 
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
% to ajust the position of the axis
maxX = max((get(ax,'XLim')));
maxY = max(abs(get(ax,'YLim')));
axis(ax,[min(knotsImgHorCoords) maxX min(knotsImgVerCoords) maxY]);

end