%% function: rawdata_save
% author: <Lin, Wei> (CC-DA/ESV1)
% date: <2017-04-11>
% save the object of cTriangulationEval which contains the raw data
% INPUTS
function rawdata_save(obj, saveDir)

save(fullfile(saveDir, 'triangEvalObj.mat'), 'obj');

end