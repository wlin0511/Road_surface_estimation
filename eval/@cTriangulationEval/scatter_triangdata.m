function scatter_triangdata(triangdata_mat, funcHandleIdx, funcHandle_name, variant)

non_invertedsign_z_indx = (triangdata_mat.invertedsign_z_flag_mat == 0);
invertedsign_z_indx = (triangdata_mat.invertedsign_z_flag_mat == 1);


figure('name',[variant,' scatter_triangdata ', funcHandle_name{funcHandleIdx}], 'NumberTitle', 'off')
% plot of error of z - distance to FoE
subplot(2,3,5);
scatter(triangdata_mat.dist_to_FoE_mat(non_invertedsign_z_indx),triangdata_mat.error_z_mat(non_invertedsign_z_indx),'.');
hold on
scatter(triangdata_mat.dist_to_FoE_mat(invertedsign_z_indx),triangdata_mat.error_z_mat(invertedsign_z_indx),'r.')
xlabel('distance to FoE')
ylabel('error of z')
title('error of z   inverted sign:red')

% plot of error of z - parallax angle
subplot(2,3,2);
scatter(triangdata_mat.parallax_angle_mat(non_invertedsign_z_indx),triangdata_mat.error_z_mat(non_invertedsign_z_indx),'.');
hold on
scatter(triangdata_mat.parallax_angle_mat(invertedsign_z_indx),triangdata_mat.error_z_mat(invertedsign_z_indx),'r.')
xlabel('parallax angle')
ylabel('error of z')
title('error of z   inverted sign:red')

% plot of error of radial distance - distance to FoE
subplot(2,3,6);
scatter(triangdata_mat.dist_to_FoE_mat(non_invertedsign_z_indx),triangdata_mat.error_radialdist_mat(non_invertedsign_z_indx),'.')
hold on
scatter(triangdata_mat.dist_to_FoE_mat(invertedsign_z_indx),triangdata_mat.error_radialdist_mat(invertedsign_z_indx),'r.')
xlabel('distance to FoE')
ylabel('error of radial distance')
title('error of radial distance   inverted sign:red')

% plot of error of radial distance - parallax angle
subplot(2,3,3);
scatter(triangdata_mat.parallax_angle_mat(non_invertedsign_z_indx),triangdata_mat.error_radialdist_mat(non_invertedsign_z_indx),'.')
hold on
scatter(triangdata_mat.parallax_angle_mat(invertedsign_z_indx),triangdata_mat.error_radialdist_mat(invertedsign_z_indx),'r.')
xlabel('parallax angle')
ylabel('error of radial distance')
title('error of radial distance   inverted sign:red')

% plot of error of invz - distance to FoE
subplot(2,3,4);
scatter(triangdata_mat.dist_to_FoE_mat(1, non_invertedsign_z_indx),triangdata_mat.error_invz_mat(1, non_invertedsign_z_indx),'.');
hold on
scatter(triangdata_mat.dist_to_FoE_mat(1, invertedsign_z_indx),triangdata_mat.error_invz_mat(1, invertedsign_z_indx),'r.')
xlabel('distance to FoE')
ylabel('error of invz')
title('error of invz   inverted sign:red')

% plot of error of inv - parallax angle
subplot(2,3,1);
scatter(triangdata_mat.parallax_angle_mat(1, non_invertedsign_z_indx),triangdata_mat.error_invz_mat(1, non_invertedsign_z_indx),'.');
hold on
scatter(triangdata_mat.parallax_angle_mat(1, invertedsign_z_indx),triangdata_mat.error_invz_mat(1, invertedsign_z_indx),'r.')
xlabel('parallax angle')
ylabel('error of invz')
title('error of invz   inverted sign:red')

end