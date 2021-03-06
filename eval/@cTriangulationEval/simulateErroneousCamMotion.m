function sim_camMotion = simulateErroneousCamMotion(obj, gtCamMotion)

sim_camMotion = cTve;
%sim_camMotion.setMotionParamsByObj(gtCamMotion);
randNValues = randn(1,6);

simCamMotionStd = obj.params.sim.camMotionStd;

sim_camMotion.setMotionParams( ...
    gtCamMotion.rotX_C2C1 + deg2rad(simCamMotionStd.rotX_deg) * randNValues(3), ...
    gtCamMotion.rotY_C2C1 + deg2rad(simCamMotionStd.rotY_deg) * randNValues(4), ...
    gtCamMotion.rotZ_C2C1 + deg2rad(simCamMotionStd.rotZ_deg) * randNValues(5), ...
    gtCamMotion.tx_C2C1_N2 + (simCamMotionStd.tx_I * obj.flowSimObj.simIntrinsics.invfu) * randNValues(1), ...
    gtCamMotion.ty_C2C1_N2 + (simCamMotionStd.ty_I * obj.flowSimObj.simIntrinsics.invfv) * randNValues(2), ...
    gtCamMotion.traveledDist + simCamMotionStd.td * randNValues(6) );

end