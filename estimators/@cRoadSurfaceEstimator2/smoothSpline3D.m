function [gridDataAfterSmoo] = ...
    smoothSpline3D(obj, tveCurrentOut, extrinsics, intrinsicObj,  imgCell, gridDataBeforeSmoo, gridWeight, variant)

            Ni = obj.params.spline.Ni;
            tol = obj.params.spline.tol;
            s = obj.params.spline.s;
            lambda = min(s,1);
            infoBoxPos = [0.14 .81 .1 .1];
            
            smoothVecV3D = 10.^(-(1:(obj.params.spline.numKnotsV3D + obj.params.spline.baseDegV3D - obj.params.spline.orderDv3D)));
                smoothVecV3D = smoothVecV3D / smoothVecV3D(1);
                smoothVecV3D = fliplr(smoothVecV3D);
%%     %%%%%%%%%%%%  L1 PSPline Smoothing Result 3D
                % pSplineFunc = @(dyb,W) pSplineSmoothingGrid2D(dyb, W, obj.params.spline);   [Y_hat, Bx, Bv, Bw, tx, tv, tw, A]
                % [Z, Bx, Bv, Bw, tx, tv, tw, A] = l1Regression3D(regressionFunc, Y, Weights, lambda, tol, Ni)
                numTimeLags = size(gridDataBeforeSmoo,3);
                pSplineFunc = @(dyb,W) pSplineSmoothingGrid3D(dyb, W, smoothVecV3D, obj.params.spline);
                [gridDataAfterSmoo, Bx3D, Bv3D, Bw3D, knotsGridCoordsX3D, knotsGridCoordsV3D, knotsGridCoordsW3D, A_3DL1]...
                    = l1Regression3D(pSplineFunc, gridDataBeforeSmoo, gridWeight, lambda, tol, Ni);
                
                infoStr3DL1Pspline ={['numHorCells=',num2str(obj.params.grid.numHorCells),' numVerCells=',num2str(obj.params.grid.numVerCells),' numTimeLags=',num2str(numTimeLags )],...
                    ['numKnotsX=',num2str(obj.params.spline.numKnotsX3D),' numKnotsV=', num2str(obj.params.spline.numKnotsV3D),' numKnotsW=', num2str(obj.params.spline.numKnotsW3D)  ],...
                    ['lambdaX=',num2str(obj.params.spline.lambdaX3D), ' lambdaV=',num2str(obj.params.spline.lambdaV3D),' lambdaW=',num2str(obj.params.spline.lambdaW3D) ],...
                    ['baseDegX=',num2str(obj.params.spline.baseDegX3D),' baseDegV=',num2str(obj.params.spline.baseDegV3D),' baseDegW=',num2str(obj.params.spline.baseDegW3D) ],...
                    [ 'orderDx=',num2str(obj.params.spline.orderDx3D),' orderDv=',num2str(obj.params.spline.orderDv3D),' orderDw=',num2str(obj.params.spline.orderDw3D) ]...
                    };
                %                 figure( 'name', 'meshGridDataBeforeSmoothing', 'NumberTitle', 'off')
                %                 obj.meshGridData( gridDataBeforeSmoo, 'k',variant)
                %                 hold on
                %                 obj.plot2DBaseFunctions(baseFuncHor,baseFuncVer, A  )
                %                 v= axis;
                %                 set(gca,'Xdir','reverse')
                %                 annotation('textbox',infoBoxPos,'String',infoStr2DL1Pspline)
                %                 title('L1 PSpline Grid data before smoothing')
                %
                %                 figure( 'name', 'meshGridDataAfterSmoothing', 'NumberTitle', 'off')
                %                 obj.meshGridData( gridDataAfterSmoo, 'k', variant)
                %                 hold on
                %                 obj.plot2DBaseFunctions(baseFuncHor,baseFuncVer, A  )
                %                 axis(v)
                %                 set(gca,'Xdir','reverse')
                %                 annotation('textbox',infoBoxPos,'String',infoStr2DL1Pspline)
                %                 title('L1 PSpline Grid data after smoothing')
                knotsImgHorCoords3D = obj.grid.cellIdxHorToCoordinates(knotsGridCoordsX3D);
                knotsImgVerCoords3D = obj.grid.cellIdxVerToCoordinates(knotsGridCoordsV3D);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot gridWeight 2D
                %                 figure( 'name', 'gridWeightOnImage', 'NumberTitle', 'off')
                %                 obj.plotGridDataOnImage( img,gridWeight,knotsImgHorCoords, knotsImgVerCoords, [], [], [min(gridWeight(:)), max(gridWeight(:))], 'Grid Weight' )
                %                 annotation('textbox',infoBoxPos,'String',infoStr2DL1Pspline)
                %                 title('Grid Weight')
                cmapLimits = [ min([gridDataBeforeSmoo(:);gridDataAfterSmoo(:)]), max([gridDataBeforeSmoo(:);gridDataAfterSmoo(:)])];
                for timelagIdx = 1:numTimeLags
                    figure( 'name', ['gridDataOnImageBeforeSmoothing timeLag', num2str(timelagIdx)], 'NumberTitle', 'off')
                    obj.plotGridDataOnImage( imgCell{1,timelagIdx},gridDataBeforeSmoo(:,:,timelagIdx),knotsImgHorCoords3D, knotsImgVerCoords3D, [], [], cmapLimits, variant )
                    annotation('textbox',infoBoxPos,'String',infoStr3DL1Pspline)
                    title(['L1 PSpline DataGridOnImageBeforeSmoothing timeLag ', num2str(timelagIdx)])
                end
                % plot gridWeight
                cmapLimitsGridWeight = [min(gridWeight(:)), max(gridWeight(:))];
                for timelagIdx = 1:numTimeLags
                    figure( 'name', ['gridWeight timeLag', num2str(timelagIdx)], 'NumberTitle', 'off')
                    obj.plotGridDataOnImage( imgCell{1,timelagIdx},gridWeight(:,:,timelagIdx),knotsImgHorCoords3D, knotsImgVerCoords3D, [], [], cmapLimitsGridWeight, 'Grid Weight' )
                    annotation('textbox',infoBoxPos,'String',infoStr3DL1Pspline)
                    title(['Grid Weight timeLag ', num2str(timelagIdx) ])
                    
                end
                for timelagIdx = 1:numTimeLags
                    figure( 'name', ['gridDataOnImageAfterSmoothing timeLag', num2str(timelagIdx)], 'NumberTitle', 'off')
                    obj.plotGridDataOnImage( imgCell{1,timelagIdx},gridDataAfterSmoo(:,:,timelagIdx), knotsImgHorCoords3D, knotsImgVerCoords3D, [], [], cmapLimits, variant)
                    annotation('textbox',infoBoxPos,'String',infoStr3DL1Pspline)
                    title(['L1 PSpline DataGridOnImageAfterSmoothing timeLag ', num2str(timelagIdx)])
                    
                    % plot 3D points
                    %                 pointsOfGridCenterBeforeSmoo = obj.compute3DCoordsOfGridCenter(intrinsicObj, gridDataBeforeSmoo, variant );
                    %                 pointsOfGridCenterAfterSmoo = obj.compute3DCoordsOfGridCenter(intrinsicObj, gridDataAfterSmoo, variant );
                    %                 cRoadSurfaceEstimator2.plot3DCoords( pointsOfGridCenterBeforeSmoo, pointsOfGridCenterAfterSmoo, [], ...
                    %                     intrinsicObj, tveCurrentOut, infoStr2DL1Pspline )
                    %                 title('L1 PSpline 3D points')
                end
                
                

end
            
            
            