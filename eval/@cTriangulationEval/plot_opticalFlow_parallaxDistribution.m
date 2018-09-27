

function plot_opticalFlow_parallaxDistribution(parallaxAngle , gt_P_I1, gt_P_I2, img_double, parallaxAngleThreshold)
if nargin > 4
    parallaxAngle(parallaxAngle>parallaxAngleThreshold) = NaN;
end
imshow(repmat(img_double,1,1,3))
hold on
plotColorLines( gt_P_I1, gt_P_I2, parallaxAngle)
colormap(flipud(jet))
cb = colorbar;
cb.Label.String = 'parallax angle [degree]';
title('parallax angle distribution in optical flow field','Interpreter','none')
set(gcf,'color','w');
end