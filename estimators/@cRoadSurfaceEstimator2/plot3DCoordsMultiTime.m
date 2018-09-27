function plot3DCoordsMultiTime(pointsBeforeSmoo, pointsAfterSmoo, intrinsicObj)

opt.imagesize = [intrinsicObj.h, intrinsicObj.w];
opt.theta = intrinsicObj.fov_u;
opt.f = 1;

numPoints = size(pointsBeforeSmoo,1)*size(pointsBeforeSmoo,2);
numTimeLags = size(pointsBeforeSmoo,4);
figure('Name','3DpointsMultiTime','NumberTitle','off')

spaceShift = 0;
for timeLagIdx = 1:numTimeLags
    pointsBeforeSmooVec = zeros(3, numPoints) ;
    for i=1:3
        pointsBeforeSmooVec(i,:) = reshape(pointsBeforeSmoo(:,:,i,timeLagIdx),1,numPoints);
    end
    pointsBeforeSmooVec(1,:) = pointsBeforeSmooVec(1,:) + spaceShift*ones(1,numPoints );
    spaceShift = spaceShift + 1e-3;
    scatter3(pointsBeforeSmooVec(1,:),pointsBeforeSmooVec(2,:), pointsBeforeSmooVec(3,:), '.',...
        'MarkerFaceAlpha', 0.5, 'MarkerEdgeAlpha',0.5);
    hold on
end
set(gca,'Ydir','reverse')
hold on 



end