function plot3DCoords(pointsBeforeSmoo, pointsAfterSmoo, P_Vehicle, extrinsics, intrinsicObj, camMotion, infoStr)


% % R_BR : roation from Body frame to Road frame (assumed identity matrix here)
% R_BR = eye(3);
% % R_CR : rotation from Camera frame to Road frame, here C refers to the normal camera frame (looking to the ground)
% R_CR = extrinsics.R_CB * R_BR;
% 
% d_C1R =  extrinsics.camHeight; % distance from the camera center of C1 to Road plane 


infoBoxPos = [0.14 .81 .1 .1];

opt.imagesize = [intrinsicObj.h, intrinsicObj.w];
opt.theta = intrinsicObj.fov_u;
opt.f = 1;

numPoints = size(pointsBeforeSmoo,1)*size(pointsBeforeSmoo,2);
pointsBeforeSmooVec = zeros(3, numPoints);
pointsAfterSmooVec = pointsBeforeSmooVec;

for i=1:3
    pointsBeforeSmooVec(i,:) = reshape(pointsBeforeSmoo(:,:,i),1,numPoints);
    pointsAfterSmooVec(i,:) = reshape(pointsAfterSmoo(:,:,i),1,numPoints);
end
xLimUp = 50; xLimDown = -50;
yLimUp = 30; yLimDown = -30;
zLimUp = 200; zLimDown = 0;

% pointsBeforeSmooFilterLogic = (pointsBeforeSmoo(:,:,2) > yLimUp) | (pointsBeforeSmoo(:,:,2) <= yLimDown) ...
%     | (pointsBeforeSmoo(:,:,3) > zLimUp) | (pointsBeforeSmoo(:,:,3) <= zLimDown);
% pointsBeforeSmooFilterLogic = (pointsBeforeSmoo(:,:,3) > zLimUp) | (pointsBeforeSmoo(:,:,3) <= zLimDown);
% pointsBeforeSmooFilterLogic = repmat(pointsBeforeSmooFilterLogic, 1,1,3);
% pointsBeforeSmooFiltered = pointsBeforeSmoo;
% pointsBeforeSmooFiltered(pointsBeforeSmooFilterLogic)=NaN;

% pointsAfterSmooFilterLogic = (pointsAfterSmoo(:,:,2) > yLimUp) | (pointsAfterSmoo(:,:,2) <= yLimDown) ...
%     | (pointsAfterSmoo(:,:,3) > zLimUp) | (pointsAfterSmoo(:,:,3) <= zLimDown);
pointsAfterSmooFilterLogic = (pointsAfterSmoo(:,:,3) > zLimUp) | (pointsAfterSmoo(:,:,3) <= zLimDown);
pointsAfterSmooFilterLogic = repmat(pointsAfterSmooFilterLogic, 1,1,3);
pointsAfterSmooFiltered = pointsAfterSmoo;
pointsAfterSmooFiltered(pointsAfterSmooFilterLogic)=NaN;


figure('Name','3Dpoints','NumberTitle','off')
scatter3(pointsBeforeSmooVec(1,:),pointsBeforeSmooVec(2,:), pointsBeforeSmooVec(3,:), 'b.',...
    'MarkerFaceAlpha', 0.5, 'MarkerEdgeAlpha',0.5);
% hold on
% scatter3(P_Vehicle(1,:),P_Vehicle(2,:), P_Vehicle(3,:), 'g.',...
%     'MarkerFaceAlpha', 0.5, 'MarkerEdgeAlpha',0.5);
% set(gca,'Ydir','reverse')

% hold on
% scatter3(pointsAfterSmooVec(1,:),pointsAfterSmooVec(2,:), pointsAfterSmooVec(3,:), 'r.',...
%     'MarkerFaceAlpha', 0.5, 'MarkerEdgeAlpha',0.5)
hold on
if size(pointsAfterSmoo, 2)==1
    plot3(pointsAfterSmooFiltered(:,:,1),pointsAfterSmooFiltered(:,:,2),pointsAfterSmooFiltered(:,:,3),'r')
else
    surface(pointsAfterSmooFiltered(:,:,1),pointsAfterSmooFiltered(:,:,2),pointsAfterSmooFiltered(:,:,3),'FaceAlpha', .2, 'FaceColor','r' )
end

hold on
draw3DCamera(opt,eye(3),[0;0;0],'r')  % camera C2
text(0,0,0,'C2','Color','red')

% draw3DCamera(opt, camMotion.R_C2C1', camMotion.t_C1C2_C1, 'g')  % camera C1
% the origin of camera 1   in camera frame 2 coordinates : P_C2 = R_C2C1 * P_C1 + t_C2C1_C2   from frame C1 to frame C2
% C1_origin = double(camMotion.t_C2C1_C2);
% text(C1_origin(1),C1_origin(2),C1_origin(3),'C1','Color','green')
xlabel('x'), ylabel('y'), zlabel('z')
axis equal
% xlim([xLimDown xLimUp]), ylim([yLimDown yLimUp]), zlim([zLimDown zLimUp]), 
hold off
annotation('textbox',infoBoxPos,'String',infoStr)





% figure('Name','3DpointsSurface','NumberTitle','off')
% surface(pointsBeforeSmooFiltered(:,:,1),pointsBeforeSmoo(:,:,2), pointsBeforeSmooFiltered(:,:,3) ,'FaceAlpha', .2, 'FaceColor','b' )
% hold on
% surface(pointsAfterSmooFiltered(:,:,1), pointsAfterSmooFiltered(:,:,2), pointsAfterSmooFiltered(:,:,3),'FaceAlpha', .2, 'FaceColor','r' )
% 
% draw3DCamera(opt,eye(3),[0;0;0],'r')  % camera C2
% text(0,0,0,'C2','Color','red')
% xlabel('x'), ylabel('y'), zlabel('z')
% axis square
% title('3D points surface')

end