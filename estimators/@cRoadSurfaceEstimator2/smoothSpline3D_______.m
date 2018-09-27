function gridDataAfterSmoo3D = smoothSpline3D_______(obj, gridDataBeforeSmoo3D, gridWeight3D )

Ni = obj.params.spline.Ni;
tol = obj.params.spline.tol;
s = obj.params.spline.s;
lambda = min(s,1);

pSplineFunc = @(dyb,W) pSplineSmoothingGrid2D(dyb, W, obj.params.spline);

gridDataBeforeSmoo2D = reshape(gridDataBeforeSmoo3D, size(gridDataBeforeSmoo3D,1), size(gridDataBeforeSmoo3D,3));
gridWeight2D = reshape(gridWeight3D, size(gridWeight3D,1), size(gridWeight3D,3));
[gridDataAfterSmoo2D, baseFuncHor, baseFuncVer, knotsGridHorCoords, knotsGridVerCoords, A] = l1Regression2D(pSplineFunc, gridDataBeforeSmoo2D, s, gridWeight2D, lambda, tol, Ni);
infoBoxPos = [0.14 .81 .1 .1];
str ={['numHorCells=',num2str(obj.params.grid.numHorCells),' numVerCells=',num2str(obj.params.grid.numVerCells)],...
    ['numKnotsX=',num2str(obj.params.spline.numKnotsX),' numKnotsV=', num2str(obj.params.spline.numKnotsV) ],...
    ['lambdaX=',num2str(obj.params.spline.lambdaX), ' lambdaV=',num2str(obj.params.spline.lambdaV)],...
    ['baseDegX=',num2str(obj.params.spline.baseDegX),' baseDegV=',num2str(obj.params.spline.baseDegV) ]...
    };
figure( 'name', 'meshGridDataBeforeSmoothing', 'NumberTitle', 'off')
obj.meshGridData( gridDataBeforeSmoo2D)
hold on
obj.plot2DBaseFunctions(baseFuncHor,baseFuncVer, A  )
annotation('textbox',infoBoxPos,'String',str)
title('Grid data before smoothing')

figure( 'name', 'meshGridDataAfterSmoothing', 'NumberTitle', 'off')
obj.meshGridData( gridDataAfterSmoo2D)
hold on
obj.plot2DBaseFunctions(baseFuncHor,baseFuncVer, A  )
annotation('textbox',infoBoxPos,'String',str)
title('Grid data after smoothing')


gridDataAfterSmoo3D = reshape(gridDataAfterSmoo2D,size(gridDataBeforeSmoo3D,1),size(gridDataBeforeSmoo3D,2), size(gridDataBeforeSmoo3D,3) );


end