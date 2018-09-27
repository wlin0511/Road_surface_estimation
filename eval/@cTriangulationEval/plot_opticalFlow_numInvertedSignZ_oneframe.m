function plot_opticalFlow_numInvertedSignZ_oneframe(invertedsign_z_flag , gt_P_I1, gt_P_I2, funcHandleIdx, img_double, funcHandle_name, frameIdx)

triangdata_invz_flag_mat = cell2mat(invertedsign_z_flag(funcHandleIdx, :, frameIdx)');
triangdata_invz_flag_sum = sum(triangdata_invz_flag_mat);
imshow(repmat(img_double,1,1,3))
hold on
plotColorLines( gt_P_I1, gt_P_I2, triangdata_invz_flag_sum)
cb = colorbar;
cb.Label.String = 'frequency';
title([funcHandle_name{funcHandleIdx}, ' frequency of inverted sign of z in all iterations in frame ', num2str(frameIdx)],'Interpreter','none')

end