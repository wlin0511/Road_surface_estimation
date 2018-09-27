classdef cRoadSurfaceEstimator2 < handle
    
    % params
    %     gridXLim = [-4, 1];
    %     gridYLim = [4, 30];
    %     numHorCells = 1;
    %     numVerCells = 20;
    %             dataMin = 1;
    %             dataMax = 2;
    %             numBins = 50;
    %             filterWingSize = 4;
    %             expLimitX = -1;
    % params.bdeg, params.pord
    % params.corrdior.extLeft, params.corrdior.extRight], params.corrdior.maxTraveledDist
    
    properties
        grid
        %         %drivingCorridor
        %         % surface state
        %         R_BS1
        %         R_BS22 % rotation body to a posteriori road plane (update)
        %         R_BS21 % rotation body to a priori road plane (prediction)
        %
        %         spline_y_S21
        %
        %         dataCircBuffer
        %         weightsCircBuffer
        
        params
        
    end
    
    methods
        function obj = cRoadSurfaceEstimator2(params)
            obj.params = params;
            obj.createGrid();
            % obj.spline_y_S21 = zeros(1, obj.grid.h);
            % obj.dataCircBuffer = CCircBuffer(params.spline.historySize); %NaN(params.spline.nimKnots+params.spline.bdeg, params.spline.historySize);
            % obj.weightsCircBuffer = CCircBuffer(params.spline.historySize); %NaN(params.spline.nimKnots+params.spline.bdeg, params.spline.historySize);
        end
        
        function createGrid(obj)
            
            % Vertical histogram grid (Z), data Y
            gridCellWidth = (obj.params.grid.gridXLim(2)-obj.params.grid.gridXLim(1)) / obj.params.grid.numHorCells;
            gridCellHeight = (obj.params.grid.gridYLim(2)-obj.params.grid.gridYLim(1)) / obj.params.grid.numVerCells;
            %
            obj.grid = cGrid( ...
                obj.params.grid.gridXLim, obj.params.grid.gridYLim, gridCellWidth, gridCellHeight, ...
                obj.params.grid.dataMin, obj.params.grid.dataMax);
        end
        
        function predict(obj, tvePrevious, tveCurrent, intrinsics, extrinsics)
            disp('Road surface: predict');
            
            % Predict driving corridor
            %tvePrevious.rotY_B2R2 = 0;
            %obj.drivingCorridor.predict(tvePrevious, extrinsics, intrinsics);
            
            % Predict new orientation camera to road surface
            Rxz_C2C1 = euler2R([tveCurrent.rotX_C2C1; 0; tveCurrent.rotZ_C2C1], false);
            obj.R_BS1 = obj.R_BS22; % last orientation body to surface was the current before
            obj.R_BS21 = Rxz_C2C1 * obj.R_BS1; % predicted orientation body to surface
        end
        
        % function [corrInlierIdx, P_C2_S] = update(obj, P_C2_S, P_C2_S_weight, tvePrevious, tveCurrent, intrinsics, extrinsics)
        function [ gridDataBeforeSmoo, gridWeight] = update(obj, P_C2_S, intrinsicObj, variant, pointsStd)
            disp('Road surface: update');
            % Clear grid
            obj.grid.clear();    % create new empty cells
            
            %P_C2 = transformFromFrame_EUC_R(P_C1, tveCurrent.R_C2C1, tveCurrent.t_C2C1_C2);
            
            % Transform 3D points from current camera frame to road
            % surface oriented frame (differ only in orientation)
            %                 R_C2S21 = extrinsics.R_CB * obj.R_BS21;
            %                 P_S21 = R_C2S21' * P_C2;
            
            % calculate inverse depth from triangluated points
            %invDepth_C2 = PC2invDepthZ(P_C2);
            
            % Sort triangluated points into grid
            
            % project the points to get image coordinates
            imgCoordsData = intrinsicObj.projPinHole(P_C2_S); % P_I2 = (X, Y)
            % grid:X,Y    data: 1/Z
            numPoints = size( P_C2_S, 2);
            if nargin < 5
                pointsStd = 1 * ones (1, numPoints); % default setting: Std = 1
            end
            
            switch variant
                case 'invZ'
                    Data = ones(1, numPoints) ./ P_C2_S(3,:);
                case 'invRadialDist'
                    Data = ones(1, numPoints) ./column_norm(P_C2_S);
                case 'Z'
                    Data = P_C2_S(3,:);
            end
            
            % put the data and weights in the cell
            obj.grid.addCorrespondences(imgCoordsData, Data, pointsStd);
            
            [gridDataBeforeSmoo, gridStd] = obj.grid.cellModesWeighted();
            assert(any(any(gridStd==0))==0, 'Std of value zero in gridStd!!') % throw error if there are zero values in gridStd
            gridWeight = 1./gridStd;
            % set the weights of grids which have no data as 0
            gridWeight (isnan(gridWeight)) = 0;
            % compute the posMidCell,  averageStd (of the points),   weightedMean and weightedStd ( of the data )
            %                 [weightedMean, weightedStd] = obj.grid.cellModesWeighted();
            
            %                 obj.grid.update();
            
            %                 figure(7900); clf;
            %                 obj.grid.plot('b');
            %                 hold on
            %                 % show text of weighted mean and weighted std of data
            %                 plot(indexDataGrid(1,gridInlierIdx), indexDataGrid(2,gridInlierIdx), '.g');
            %                 gridSize = size(obj.grid.DataGrid);
            %                 for j = 1 : gridSize(1)
            %                     for i = 1 : gridSize(2)
            %                         [xPosMidCell, yPosMidCell] = obj.grid.cellIdxToCoordinates(i, j);
            %                         text(xPosMidCell,yPosMidCell, ...
            %                             [num2str(weightedMean(j,i)),',', num2str(weightedStd(j,i) )], 'color','b');
            %                     end
            %                 end
            %                 xlabel('x [pixel]');
            %                 ylabel('y [pixel]');
            %                 title([variant, ' (weightedMean, weightedStd(of invZ))']);
            
            
            
        end
        
        
        
        function [gridDataAfterSmoo] ...
                = smoothSpline(obj, tveCurrentOut, extrinsics, intrinsicObj,  img, gridDataBeforeSmoo, gridWeight, variant)
            
            Ni = obj.params.spline.Ni;
            tol = obj.params.spline.tol;
            s = obj.params.spline.s;  % used as the smoothing coefficient of the L2 part in L1 P-spline
            lambda_1DL2 = obj.params.spline.lambda_1DL2; % used only for L2 P-spline in 1D case
            lambda = min(s,1);  %  used in shrinkage function for L1 P-spline  in  1D, 2D and 3D  case 
            infoBoxPos = [0.14 .81 .1 .1];
            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% dct L1 smoothing
            gridWeightForL1DCT = ones(size(gridDataBeforeSmoo ));
            gridWeightForL1DCT( isnan(gridDataBeforeSmoo) ) = 0;
            L1DCT_s = 1000;
            gridDataAfterSmooDct = l1SplineSmoothingND(gridDataBeforeSmoo, L1DCT_s, gridWeightForL1DCT , lambda, tol, Ni);
            infoStrDCT = {['numHorCells=',num2str(obj.params.grid.numHorCells),' numVerCells=',num2str(obj.params.grid.numVerCells)],...
                ['lambda(in shrinkage)=', num2str(lambda)],...
                ['s(in L2 part)=',num2str(L1DCT_s)],...
                };
            
            if isvector(gridDataBeforeSmoo)   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  1D case
                gridDataBeforeSmoo(60) = 70;
                gridDataBeforeSmoo(127)=80;
                gridDataBeforeSmoo(250)=60;
                gridDataBeforeSmoo(288)=NaN;
                gridDataBeforeSmoo(287)=NaN;
                gridDataBeforeSmoo(286)=NaN;
                gridDataBeforeSmoo(181:190)=NaN;
                
                numKnots = obj.params.spline.numKnots;
                bdeg = obj.params.spline.bdeg;
                pord = obj.params.spline.pord;
                z_S21 = obj.grid.cellIdxVerToCoordinates();
                % v  size: 1*(numKnots + bdeg - pord)  is the vector for variable smoothness control 
                variableSmoothingScale1D = 2.^(-(1:(numKnots + bdeg - pord)));
                variableSmoothingScale1D = variableSmoothingScale1D/variableSmoothingScale1D(1);
                variableSmoothingScale1D = fliplr(variableSmoothingScale1D);
                 
                
                % [yhat, gcv, a, B, t] = pSplineSmoothingEquiDistWeighted1D(x_data, y_data, weights, ndx, bdeg, pord, lambda)
                pSplineFunc = @(dyb,w) pSplineSmoothingEquiDistWeighted1D(z_S21, dyb, w, variableSmoothingScale1D, numKnots, bdeg, pord, s);
                [gridDataAfterSmoo, gcv, A_1DL1, baseFunc1D, knotsCoords1D] = l1Regression1D(pSplineFunc, gridDataBeforeSmoo', [], gridWeight', lambda, tol, Ni);
                % [obj.spline_y_S21, gcv, a, B, t] = l1Regression1D(pSplineFunc, grid_y_S21', s, w', lambda, tol, Ni);
                disp(['1D L1 psline smoothing, lambda(in L1 part for shrinkage) = ', num2str(lambda),', lambda(in L2 part, also  s) = ', num2str(s), ', gcv = ', num2str(gcv)])
                %%     %%%%%%%%%%%% plot L1 P-spline Smoothing Result 1D
                infoStr1DL1Pspline ={['numHorCells=',num2str(obj.params.grid.numHorCells),' numVerCells=',num2str(obj.params.grid.numVerCells)],...
                    ['numKnots=',num2str(obj.params.spline.numKnots +1) ],...
                    ['lambda(in shrinkage)=', num2str(lambda)],...
                    ['s(in L2 part)=',num2str(obj.params.spline.s)],...
                    ['baseDeg=',num2str(obj.params.spline.bdeg) ],...
                    [ 'orderD=',num2str(obj.params.spline.pord)]...
                    };
                figure('name', 'SmoothingResult1D', 'NumberTitle', 'off')
                obj.meshGridData( gridDataBeforeSmoo, 'k',variant)
                hold on
                scatterH = obj.meshGridData( gridDataAfterSmoo', 'r', variant);
                scatterH.MarkerEdgeAlpha = 0.5;
                scatterH.MarkerFaceAlpha = 0.5;
                % obj.plot2DBaseFunctions(baseFuncHor,baseFuncVer, A  )
                obj.plot1DBaseFunctions( baseFunc1D, A_1DL1  )
                set(gca,'Xdir','reverse')
                annotation('textbox',infoBoxPos,'String',infoStr1DL1Pspline)
                title('L1 PSpline Smoothing Result 1D')
                %%          %%%%%%%%%%%% plot L1 DCT Smoothing Result 1D
                figure('name', 'SmoothingResult1Ddct', 'NumberTitle', 'off')
                obj.meshGridData( gridDataBeforeSmoo, 'k',variant)
                hold on
                scatterH = obj.meshGridData( gridDataAfterSmooDct, 'r', variant);
                scatterH.MarkerEdgeAlpha = 0.5;
                scatterH.MarkerFaceAlpha = 0.5;
                set(gca,'Xdir','reverse')
                annotation('textbox',infoBoxPos,'String',infoStrDCT)
                title('L1 DCT Smoothing Result of 1D')
                
                %%     %%%%%%%%%%%%%%%%%%%%% L2 Spline Smoothing 1D
                infoStr1DL2 ={['numHorCells=',num2str(obj.params.grid.numHorCells),' numVerCells=',num2str(obj.params.grid.numVerCells)],...
                    ['numKnots=',num2str(obj.params.spline.numKnots+1) ],...
                    ['lambda=',num2str(obj.params.spline.lambda_1DL2)],...
                    ['baseDeg=',num2str(obj.params.spline.bdeg) ],...
                    [ 'orderD=',num2str(obj.params.spline.pord)]...
                    };
                [gridDataAfterSmoo1DL2, gcv1DL2, A_1DL2, baseFuncL2, knotsL2] = pSplineSmoothingEquiDistWeighted1D(z_S21, gridDataBeforeSmoo', gridWeight', variableSmoothingScale1D, numKnots, bdeg, pord, lambda_1DL2);
                
                disp(['1D L2 spline smoothing  lambda=', num2str(obj.params.spline.lambda_1DL2),',  gcv1DL2=', num2str(gcv1DL2) ])
                % pSplineFunc = @(dyb,w) pSplineSmoothingEquiDistWeighted1D(z_S21, dyb, w, numKnots, bdeg, pord, s);
                figure('name', 'L2SmoothingResult1D', 'NumberTitle', 'off')
                obj.meshGridData( gridDataBeforeSmoo, 'k',variant)
                hold on
                scatterH = obj.meshGridData( gridDataAfterSmoo1DL2', 'r', variant);
                scatterH.MarkerEdgeAlpha = 0.5;
                scatterH.MarkerFaceAlpha = 0.5;
                % obj.plot2DBaseFunctions(baseFuncHor,baseFuncVer, A  )
                obj.plot1DBaseFunctions( baseFuncL2, A_1DL2  )
                set(gca,'Xdir','reverse')
                annotation('textbox',infoBoxPos,'String',infoStr1DL2)
                title('L2 Spline Smoothing Result of 1D')
            elseif ismatrix(gridDataBeforeSmoo) && size(gridDataBeforeSmoo,1) > 1 &&  size(gridDataBeforeSmoo,2) > 1 
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 2D case
                %                 gridWeight(gridWeight>10000)=10000;
                %                 gridWeight(gridWeight~=0)=10000;
                smoothVecX = ones(1, obj.params.spline.numKnotsX + obj.params.spline.baseDegX - obj.params.spline.orderDx);
%                 vecLengthX = obj.params.spline.numKnotsX + obj.params.spline.baseDegX - obj.params.spline.orderDx;
%                 smoothVecMedian = median(1:vecLengthX);
%                 smoothVecX = ((1:vecLengthX)-smoothVecMedian).^2+1;
                
                smoothVecV = 10.^(-(1:(obj.params.spline.numKnotsV + obj.params.spline.baseDegV - obj.params.spline.orderDv)));
                smoothVecV = smoothVecV / smoothVecV(1);
                smoothVecV = fliplr(smoothVecV);
%                 smoothVecV = ones(1, obj.params.spline.numKnotsV + obj.params.spline.baseDegV - obj.params.spline.orderDv);
                %%    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  L1 PSpline Smoothing  2D
                pSplineFunc = @(dyb,W) pSplineSmoothingGrid2D(dyb, W, smoothVecX, smoothVecV, obj.params.spline);
                
                [gridDataAfterSmoo, baseFuncHor, baseFuncVer, knotsGridHorCoords2D, knotsGridVerCoords2D, A_2DL1]...
                    = l1Regression2D(pSplineFunc, gridDataBeforeSmoo, [] , gridWeight, lambda, tol, Ni);
                
                infoStr2DL1Pspline ={['numHorCells=',num2str(obj.params.grid.numHorCells),' numVerCells=',num2str(obj.params.grid.numVerCells)],...
                    ['numKnotsX=',num2str(obj.params.spline.numKnotsX+1),' numKnotsV=', num2str(obj.params.spline.numKnotsV+1) ],...
                    ['lambdaX=',num2str(obj.params.spline.lambdaX), ' lambdaV=',num2str(obj.params.spline.lambdaV)],...
                    ['baseDegX=',num2str(obj.params.spline.baseDegX),' baseDegV=',num2str(obj.params.spline.baseDegV) ],...
                    [ 'orderDx=',num2str(obj.params.spline.orderDx),' orderDv=',num2str(obj.params.spline.orderDv)]...
                    };
%                 % mesh gidData before smoothing L1 Spline
%                 figure( 'name', 'meshGridDataBeforeSmoothing', 'NumberTitle', 'off')
%                 obj.meshGridData( gridDataBeforeSmoo, 'k',variant)
%                 hold on
%                 obj.plot2DBaseFunctions(baseFuncHor,baseFuncVer, A_2DL1  )
%                 v= axis;
%                 set(gca,'Xdir','reverse')
%                 annotation('textbox',infoBoxPos,'String',infoStr2DL1Pspline)
%                 title('L1 PSpline Grid data before smoothing')
%                 % mesh gidData after smoothing L1 spline
%                 figure( 'name', 'meshGridDataAfterSmoothing', 'NumberTitle', 'off')
%                 obj.meshGridData( gridDataAfterSmoo, 'k', variant)
%                 hold on
%                 obj.plot2DBaseFunctions(baseFuncHor,baseFuncVer, A_2DL1  )
%                 axis(v)
%                 set(gca,'Xdir','reverse')
%                 annotation('textbox',infoBoxPos,'String',infoStr2DL1Pspline)
%                 title('L1 PSpline Grid data after smoothing')
                
                knotsImgHorCoords = obj.grid.cellIdxHorToCoordinates(knotsGridHorCoords2D);
                knotsImgVerCoords = obj.grid.cellIdxVerToCoordinates(knotsGridVerCoords2D);
                
                cmapLimits = [ min(min([gridDataBeforeSmoo,gridDataAfterSmoo])), max(max([gridDataBeforeSmoo,gridDataAfterSmoo]))];
                % gridData on image before smoothing L1 spline
                figure( 'name', 'gridDataOnImageBeforeSmoothing', 'NumberTitle', 'off')
                obj.plotGridDataOnImage( img,gridDataBeforeSmoo,knotsImgHorCoords, knotsImgVerCoords, baseFuncHor, baseFuncVer, cmapLimits, variant )
                annotation('textbox',infoBoxPos,'String',infoStr2DL1Pspline)
                title('L1 PSpline DataGridOnImageBeforeSmoothing')
                % gridData on image after smoothing L1 spline
                figure( 'name', 'gridDataOnImageAfterSmoothing', 'NumberTitle', 'off')
                obj.plotGridDataOnImage( img,gridDataAfterSmoo, knotsImgHorCoords, knotsImgVerCoords, baseFuncHor, baseFuncVer, cmapLimits, variant)
                annotation('textbox',infoBoxPos,'String',infoStr2DL1Pspline)
                title('L1 PSpline DataGridOnImageAfterSmoothing')
                % plot 3D points L1 spline
                pointsOfGridCenterBeforeSmoo = obj.compute3DCoordsOfGridCenter(intrinsicObj, gridDataBeforeSmoo, variant );
                pointsOfGridCenterAfterSmoo = obj.compute3DCoordsOfGridCenter(intrinsicObj, gridDataAfterSmoo, variant );
                cRoadSurfaceEstimator2.plot3DCoords( pointsOfGridCenterBeforeSmoo, pointsOfGridCenterAfterSmoo, [], ...
                    extrinsics, intrinsicObj, tveCurrentOut, infoStr2DL1Pspline )
                title('L1 PSpline 3D points')
                
                %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot colorGrid Y
                cmapLimitsColorGridY = [ min(min([pointsOfGridCenterBeforeSmoo(:,:,2),pointsOfGridCenterAfterSmoo(:,:,2)])),...
                    max(max([pointsOfGridCenterBeforeSmoo(:,:,2),pointsOfGridCenterAfterSmoo(:,:,2)]))];
                figure('name','colorGridY L1Spline BeforeSmoo', 'NumberTitle', 'off')
                obj.plotGridDataOnImage( img, pointsOfGridCenterBeforeSmoo(:,:,2),knotsImgHorCoords, knotsImgVerCoords, [], [], cmapLimitsColorGridY, 'Y' )
                annotation('textbox',infoBoxPos,'String',infoStr2DL1Pspline)
                title('L1 PSpline Clolor Grid Y before smoothing')
                figure('name','colorGridY L1Spline BeforeSmoo', 'NumberTitle', 'off')
                obj.plotGridDataOnImage( img, pointsOfGridCenterAfterSmoo(:,:,2),knotsImgHorCoords, knotsImgVerCoords, [], [], cmapLimitsColorGridY, 'Y' )
                annotation('textbox',infoBoxPos,'String',infoStr2DL1Pspline)
                title('L1 PSpline Clolor Grid Y after smoothing')
                
                %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot gridWeight 2D
                figure( 'name', 'gridWeightOnImage', 'NumberTitle', 'off')
                obj.plotGridDataOnImage( img,gridWeight,knotsImgHorCoords, knotsImgVerCoords, [], [], [min(gridWeight(:)), max(gridWeight(:))], 'Grid Weight' )
                annotation('textbox',infoBoxPos,'String',infoStr2DL1Pspline)
                title('Grid Weight')
                %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot L1 DCT Smoothing Result 2D
                cmapLimitsDCT = [ min(min([gridDataBeforeSmoo,gridDataAfterSmooDct])), max(max([gridDataBeforeSmoo,gridDataAfterSmooDct]))];
                % gridData on image before smoothing L1 DCT
                figure( 'name', 'gridDataOnImageBeforeSmoothingDCT', 'NumberTitle', 'off')
                obj.plotGridDataOnImage( img,gridDataBeforeSmoo,knotsImgHorCoords, knotsImgVerCoords, [], [], cmapLimitsDCT, variant )
                annotation('textbox',infoBoxPos,'String',infoStrDCT)
                title('L1 DCT DataGridOnImageBeforeSmoothing DCT')
                % gridData on image after smoothing L1 DCT
                figure( 'name', 'gridDataOnImageAfterSmoothingDCT', 'NumberTitle', 'off')
                obj.plotGridDataOnImage( img,gridDataAfterSmooDct, knotsImgHorCoords, knotsImgVerCoords, [], [], cmapLimitsDCT, variant)
                annotation('textbox',infoBoxPos,'String',infoStrDCT)
                title('L1 DCT DataGridOnImageAfterSmoothing DCT')
                
                % plot 3D points L1 DCT
                % pointsOfGridCenterBeforeSmoo = obj.compute3DCoordsOfGridCenter(intrinsicObj, gridDataBeforeSmoo, variant );
                pointsOfGridCenterAfterSmooDCT = obj.compute3DCoordsOfGridCenter(intrinsicObj, gridDataAfterSmooDct, variant );
                cRoadSurfaceEstimator2.plot3DCoords( pointsOfGridCenterBeforeSmoo, pointsOfGridCenterAfterSmooDCT, [], ...
                    extrinsics, intrinsicObj, tveCurrentOut, infoStrDCT )
                title('L1 DCT 3D points DCT')
                %%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% L2 Spline Smoothing 2D
                infoStr2DL2 ={['numHorCells=',num2str(obj.params.grid.numHorCells),' numVerCells=',num2str(obj.params.grid.numVerCells)],...
                    ['numKnotsX=',num2str(obj.params.spline.numKnotsX),' numKnotsV=', num2str(obj.params.spline.numKnotsV) ],...
                    ['lambdaX=',num2str(obj.params.spline.lambdaX), ' lambdaV=',num2str(obj.params.spline.lambdaV)],...
                    ['baseDegX=',num2str(obj.params.spline.baseDegX),' baseDegV=',num2str(obj.params.spline.baseDegV) ],...
                    [ 'orderDx=',num2str(obj.params.spline.orderDx),' orderDv=',num2str(obj.params.spline.orderDv)]...
                    };
                
                % pSplineFunc = @(dyb,W) pSplineSmoothingGrid2D(dyb, W, obj.params.spline);
                % Prepare data weights
                W = isfinite(gridDataBeforeSmoo);
                if ~isempty(gridWeight)
                    W = W .* gridWeight;
                end
                gridDataBeforeSmoo_ = gridDataBeforeSmoo;
                gridDataBeforeSmoo_(W==0) = 0; % arbitrary value for missing data
                [gridDataAfterSmoo2DL2, baseFuncXL2 , baseFuncVL2, knotsX, knotsV, A_2DL2] = pSplineSmoothingGrid2D(gridDataBeforeSmoo_, W, smoothVecX, smoothVecV, obj.params.spline);
                % mesh gidData before smoothing L2 Spline
                figure( 'name', 'meshGridDataBeforeSmoothing', 'NumberTitle', 'off')
                obj.meshGridData( gridDataBeforeSmoo, 'k',variant)
                hold on
                obj.plot2DBaseFunctions(baseFuncXL2,baseFuncVL2,  A_2DL2  )
                v= axis;
                set(gca,'Xdir','reverse')
                annotation('textbox',infoBoxPos,'String',infoStr2DL2)
                title('L2 PSpline Grid data before smoothing')
                % mesh gidData after smoothing L2 Spline
                figure( 'name', 'meshGridDataAfterSmoothing', 'NumberTitle', 'off')
                obj.meshGridData( gridDataAfterSmoo2DL2, 'k', variant)
                hold on
                obj.plot2DBaseFunctions(baseFuncXL2,baseFuncVL2, A_2DL2  )
                axis(v)
                set(gca,'Xdir','reverse')
                annotation('textbox',infoBoxPos,'String',infoStr2DL2)
                title('L2 PSpline Grid data after smoothing')
                
                cmapLimitsL2 = [ min(min([gridDataBeforeSmoo,gridDataAfterSmoo2DL2])), max(max([gridDataBeforeSmoo,gridDataAfterSmoo2DL2]))];
                % gridData on image before smoothing L2 spline
                figure( 'name', 'gridDataOnImageBeforeSmoothing', 'NumberTitle', 'off')
                obj.plotGridDataOnImage( img,gridDataBeforeSmoo,knotsImgHorCoords, knotsImgVerCoords, baseFuncXL2, baseFuncVL2, cmapLimitsL2, variant )
                annotation('textbox',infoBoxPos,'String',infoStr2DL2)
                title('L2 PSpline DataGridOnImageBeforeSmoothing')
                % gridData on image after smoothing L2 spline
                figure( 'name', 'gridDataOnImageAfterSmoothing', 'NumberTitle', 'off')
                obj.plotGridDataOnImage( img,gridDataAfterSmoo2DL2, knotsImgHorCoords, knotsImgVerCoords, baseFuncXL2, baseFuncVL2, cmapLimitsL2, variant)
                annotation('textbox',infoBoxPos,'String',infoStr2DL2)
                title('L2 PSpline DataGridOnImageAfterSmoothing')
                % plot 3D points L2 spline
                % pointsOfGridCenterBeforeSmoo = obj.compute3DCoordsOfGridCenter(intrinsicObj, gridDataBeforeSmoo, variant );
                pointsOfGridCenterAfterSmoo2DL2 = obj.compute3DCoordsOfGridCenter(intrinsicObj, gridDataAfterSmoo2DL2, variant );
                cRoadSurfaceEstimator2.plot3DCoords( pointsOfGridCenterBeforeSmoo, pointsOfGridCenterAfterSmoo2DL2, [], ...
                    extrinsics, intrinsicObj, tveCurrentOut, infoStr2DL2)
                title('L2 PSpline 3D points')    
            else
                error('incorrect input dimensions!')
            end
            
            %             smoothingErr = (gridDataAfterSmoo - gridDataBeforeSmoo);
            %             smoothingErr(isnan(smoothingErr)) = 0;
            %             gridWeightNormalized = gridWeight/ sum(gridWeight(:));
            %             sumOfSquaredErr = sum(sum((smoothingErr.^2).*gridWeightNormalized));
            %             disp(['L1 PSpline smoothing error = ', num2str(sumOfSquaredErr)])
            
            
            return
            %%
            % Calculate normal of the road surface in predicted surface frame S21
            z_normal = 8; % dist. for what the normal should be calculated
            t_normal = obj.dataCircBuffer.size - 0.01; % time step for what the normal should be calculated (current)
            [p_tzy, t_ty, t_zy, n_tzy] = bsplineTangentAndNormal2D(t_normal, z_normal, A, knotsGridHorCoords, knotsGridVerCoords, 2, 2);
            
            n_S21 = vec2unitVec([0; n_tzy(3); n_tzy(2)]);
            
            % Update tve-array
            e_S21S22 = [n_S21(3); 0; -n_S21(1)];
            R_S21S22 = euler2R(e_S21S22, true);
            R_BS22 = obj.R_BS21 * R_S21S22;
            e_BS22 = R2euler(R_BS22);
            tveCurrentOut.rotX_B2R2 = e_BS22(1);
            tveCurrentOut.rotY_B2R2 = 0;
            tveCurrentOut.rotZ_B2R2 = e_BS22(3);
            
            h = - n_S21' * ([0;0;0] - [0; p_tzy(3); p_tzy(2)]);
            tveCurrentOut.d = h;
            
            
            [dB, da, ddBdda] = evaluateBSplineSecondDerivative1D(z_normal, knotsGridVerCoords, A(:,end), obj.params.spline.baseDegX);
            tveCurrentOut.c0 = ddBdda;
            
            
            
            figure(345)
            clf;
            surf(gridDataAfterSmoo);
            hold on;
            X = repmat(1:size(in_Y_S21,2), size(in_Y_S21,1), 1);
            Y = repmat((1:size(in_Y_S21,1))', 1, size(in_Y_S21,2));
            scatter3(X(:), Y(:), in_Y_S21(:), 20, in_Y_S21(:), 'filled');
            %surf(in_Y_S21, 'FaceAlpha',0.1);
            for i = 1 : size(in_Y_S21,2)
                plot3(ones(size(in_Y_S21,1))*i, 1:size(in_Y_S21,1), in_Y_S21(:,i)', '-k');
            end
            %                 plotLine3(p_tzy, p_tzy+n_tzy, 'r');
            %                 plotLine3(p_tzy, p_tzy+t_ty, 'g');
            %                 plotLine3(p_tzy, p_tzy+t_zy, 'b');
            set(gca,'Zdir','reverse');
            xlabel('t'), ylabel('z'), zlabel('y');
            %axis equal
            axis vis3d
            colormap default
            title('Vertical Surface Smooting Over Time');
            %caxis([0.5, 3]);
            
            obj.spline_y_S21 = gridDataAfterSmoo(:,end)';
            %             else
            %                 return
            
            
            % Calculate normal of the road surface in predicted surface frame S21
            z_normal = 8; %t(2) + 0.001; % dist. for what the normal should be calculated
            [p_zy, t_zy, n_zy] = bsplineTangentAndNormal1D(z_normal, a, t, 2);
            n_S21 = vec2unitVec([0; n_zy(2); n_zy(1)]);
            
            % From the normal we get the delta rotation to the current
            % surface planar frame S22, which is then used to update R_BS22
            e_S21S22 = [n_S21(3); 0; -n_S21(1)];
            
            
            % Calculate height over ground plane
            h = bsplineDistOriginApprox(a, t, 2);
            
            
            %             % Update tve-array
            %             R_S21S22 = euler2R(e_S21S22, true);
            %             R_BS22 = obj.R_BS21 * R_S21S22;
            %             e_BS22 = R2euler(R_BS22);
            %             tveCurrentOut.rotX_B2R2 = e_BS22(1);
            %             tveCurrentOut.rotY_B2R2 = 0;
            %             tveCurrentOut.rotZ_B2R2 = e_BS22(3);
            
            % hack
            obj.R_BS22 = eye();
            
            %             % 2D plot of vertical spline fits
            %             gridCoordX = 1 : obj.grid.w;
            %             gridCoordY = 1 : obj.grid.h;
            %             [dataCoordX, dataCoordZ] = obj.grid.cellIdxToCoordinates(gridCoordX, gridCoordY);
            
            %     % Plot
            %     figure(4733);
            %     clf;
            %     hold on;
            %     plot(iZ_gridRows, z_p1, '.-c');
            %     plot(p_izy(1), p_izy(2), '*b');
            %     plotLine(p_izy-t_izy, p_izy+t_izy, 'b');
            %     plotLine(p_izy, p_izy+n_izy, 'r');
            %     %plotLine([(1/zNormal)-1; yTangent-iz_slopeTangent], [(1/zNormal)+1; yTangent+iz_slopeTangent], 'r');
            %     %plotLine([(1/zNormal)-1; yTangent+(1/iz_slopeTangent)], [(1/zNormal)+1; yTangent-(1/iz_slopeTangent)], 'c');
            %     axis equal
            
            figure(477);
            clf;
            hold on;
            %plot(P_C2(3,gridInlierIdx), P_C2(2,gridInlierIdx), '.g');
            % Plot mean values (nan as zero)
            plot(z_S21, gridDataBeforeSmoo, '*b');
            idxNoMean = find(isnan(gridDataBeforeSmoo));
            defaultMean = zeros(size(gridDataBeforeSmoo));
            plot(z_S21(idxNoMean), defaultMean(idxNoMean), '*r');
            %
            %plot(depthCoordY, l1Surf, '.-b');
            %plot(depthCoordY, pSurf, '.-k');
            plot(t, zeros(size(t)), 'og');
            plot(z_S21, obj.spline_y_S21, '.-c');
            plot(p_zy(1), p_zy(2), '*k');
            plotLine(p_zy, p_zy+n_zy, 'r');
            plotLine(p_zy-t_zy, p_zy+t_zy, 'b');
            
            plotCos3(eye(3), [0;0;0], 1, 'b');
            plotCos3(extrinsics.R_CB, [0;0;0], 0.5, 'r');
            %plotCos3(R_BS22, [0;0;0], 0.5, 'c');
            
            
            % plot closest point on surface
            pClosesest_S21 = n_S21 * h;
            plot(pClosesest_S21(1), pClosesest_S21(2), '.r');
            
            %
            % Plot basis
            %[B, t] = createBSplineBasisEquiDist(x_data, xl, xr, ndx, bdeg);
            samplePosB = min(t)+0.1 : 0.1 : max(t);
            B_plot = evaluateBSplineBasisEquiDist(samplePosB, t, bdeg);
            for splIdx = 1 : size(B_plot,2)
                h3 = plot(samplePosB, B_plot(:,splIdx)); %*a(splIdx));
            end
            %
            %legend('raw data', 'grid cell mean', 'l1 fit', 'p2 fit', 'p1 fit');%, 'p-spline fit');
            axis equal tight;
            set(gca,'Ydir','reverse');
            ylim([-3, 5]);
            title('Vertical (Z) P1');
            xlabel('z'), ylabel('y');
        end
        
        function plot(obj, intrinsics, extrinsics, color)
            
            % plot surface (vertical grid)
            halfCellWidth = (obj.grid.horCellSize)*0.5;
            xLeft = obj.grid.cellIdxHorToCoordinates(1) - halfCellWidth;
            
            
            
            
            xRight = obj.grid.cellIdxHorToCoordinates(obj.grid.w) + halfCellWidth;
            xMid = xLeft + (xRight - xLeft) * 0.5;
            z_S21 = obj.grid.cellIdxVerToCoordinates();
            
            mp_R1 = obj.drivingCorridor.getCorridorMidPointForTraveledDist(z_S21);
            dx = mp_R1(1,:);
            
            cellPointsMid_S22 = [repmat(xMid,1,obj.grid.h) + dx; obj.spline_y_S21; z_S21];
            cellPointsLeft_S22 = [repmat(xLeft,1,obj.grid.h) + dx; obj.spline_y_S21; z_S21];
            cellPointsRight_S22 = [repmat(xRight,1,obj.grid.h) + dx; obj.spline_y_S21; z_S21];
            
            R_C2S22 = extrinsics.R_CB * obj.R_BS22;
            cellPointsMid_C2 = R_C2S22 * cellPointsMid_S22;
            cellPointsLeft_C2 = R_C2S22 * cellPointsLeft_S22;
            cellPointsRight_C2 = R_C2S22 * cellPointsRight_S22;
            
            cellPointsMid_I2 = intrinsics.projPinHole(cellPointsMid_C2);
            cellPointsLeft_I2 = intrinsics.projPinHole(cellPointsLeft_C2);
            cellPointsRight_I2 = intrinsics.projPinHole(cellPointsRight_C2);
            
            hold on;
            
            %             % plot triangulated points
            %             p_I2 = intrinsics.projPinHole(P_C2);
            %             plot(p_I2(1,:), p_I2(2,:), '.g');
            
            %             scatter(cellPointsMid_I2(1,:), cellPointsMid_I2(2,:), 5, color, 'filled');
            %             scatter(cellPointsLeft_I2(1,:), cellPointsLeft_I2(2,:), 5, color, 'filled');
            %             scatter(cellPointsRight_I2(1,:), cellPointsRight_I2(2,:), 5, color, 'filled');
            
            plot(cellPointsMid_I2(1,:), cellPointsMid_I2(2,:), 'color',color);
            plot(cellPointsLeft_I2(1,:), cellPointsLeft_I2(2,:), 'color',color);
            plot(cellPointsRight_I2(1,:), cellPointsRight_I2(2,:), 'color',color);
            
            plotLines(cellPointsLeft_I2, cellPointsRight_I2, color);
            
        end
        
        
        plotGridDataOnImage(obj, img, DataGrid, knotsImgHorCoords, knotsImgVerCoords, baseFuncHor, baseFuncVer, cmapLimits, variant  )
        
        plot1DBaseFunctions(obj, baseFuncVer, A  )
        plot2DBaseFunctions(obj, baseFuncHor,baseFuncVer, A  )
        %% function : meshGridData
        plotHandle = meshGridData(obj, gridData, color, variant)
        
        %% function : update3D
        [ gridDataBeforeSmoo, gridWeight] = update3D(obj, P_C2_S_cell, intrinsicObj, variant, pointsStd)
        
        %% function updateMerge
        [ gridDataBeforeSmoo, gridWeight] = updateMerge(obj, P_C2_S_cell, intrinsicObj, variant, pointsStd_cell)
        
        %% function compute3DCoordsOfGridCenter
        P_C = compute3DCoordsOfGridCenter(obj, intrinsicObj, gridData, variant )
        %% function smoothSpline3D
        [gridDataAfterSmoo] = ...
            smoothSpline3D(obj, tveCurrentOut, extrinsics, intrinsicObj,  imgCell, gridDataBeforeSmoo, gridWeight, variant)
        
    end
    
    methods(Static)
        plot3DCoords(pointsBeforeSmoo, pointsAfterSmoo, P_Vehicle, extrinsics, intrinsicObj, camMotion, infoStr)
        
        plot3DCoordsMultiTime(pointsBeforeSmoo, pointsAfterSmoo, intrinsicObj)
        
        [robustGridData, robustGridWeight] = smoothingRANSAC(numIter, numToRemove, smoothingErr, tolerance, pSplineFunc, gridDataBeforeSmoo, s, gridWeight, lambda, tol, Ni)
    end
    
    
    
end