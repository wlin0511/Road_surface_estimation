function [robustGridData, robustGridWeight] = smoothingRANSAC(numIter, numToRemove, smoothingErr, tolerance, pSplineFunc, gridDataBeforeSmoo, s, gridWeight, lambda, tol, Ni)


% [gridDataAfterSmoo, baseFuncHor, baseFuncVer, knotsGridHorCoords, knotsGridVerCoords, A] ...
%     = l1Regression2D(pSplineFunc, gridDataBeforeSmoo, s, gridWeight, lambda, tol, Ni);
numGrid = numel(gridDataBeforeSmoo);
gridDataBeforeSmooVec = gridDataBeforeSmoo(:);
gridWeightVec = gridWeight(:);
[m,n] = size(gridDataBeforeSmoo);
conSet = cell(numIter,1); % the consensus sets in every iteration
conSetNumber = zeros(numIter,1);% number of inliers in every iteration
errSum = zeros(numIter,1); % the sum of errors in every iteration
for i = 1:numIter
    
    randSeq = randperm(numGrid);
    idxToRemove = randSeq(1:numToRemove);
    dataChosen = gridDataBeforeSmooVec;
    gridWeightChosen = gridWeightVec;
    dataChosen(idxToRemove) = NaN;
    gridWeightChosen(idxToRemove) = 0;
    dataChosen = reshape(dataChosen, m,n);
    gridWeightChosen = reshape(gridWeightChosen, m,n);
    [gridDataAfterSmoo, ~, ~, ~, ~, ~] ...
        = l1Regression2D(pSplineFunc, dataChosen, s, gridWeightChosen, lambda, tol, Ni);
    smoothingErr = abs(gridDataAfterSmoo - gridDataBeforeSmoo);
    smoothingErrVec = smoothingErr(:);
    conSet{i,1} = find(smoothingErrVec <= tolerance);
    conSetNumber(i,1) = numel(conSet{i,1});
    errSum(i,1) = sum(smoothingErrVec(conSet{i,1}));
    
end

[maxNumber,~]=max(conSetNumber);
maxNumberIdx = find(conSetNumber ==maxNumber);
if length(maxNumberIdx )>1
   errSumMax = errSum(maxNumberIdx,:);
   [~,minErrSumIdx]=min(errSumMax);
    consensusSet = conSet{maxNumberIdx(minErrSumIdx),1};
else
    consensusSet = conSet{maxNumberIdx,1};
end
robustGridDataVec = NaN(1,numGrid);
robustGridDataVec(consensusSet) = gridDataBeforeSmooVec(consensusSet);
robustGridData = reshape(robustGridDataVec,m,n);

robustGridWeight = gridWeight;
robustGridWeight(isnan(robustGridData))=0;
end