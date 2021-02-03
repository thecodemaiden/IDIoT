function scores = getImu2BodyScores(dataIMU, dataCam, tW, tWindowOverlap, decFactor)
if nargin < 4 || isempty(tWindowOverlap), tWindowOverlap = 0.5; end
if nargin < 5 || isempty(decFactor), decFactor = 1; end

t = (0:(length(dataCam.camPos(1).rWrist.pos2D)-1))./dataCam.fps;
scores = cell(1,1);
iSegment = 1;
while true
    tS = (iSegment-1)*tW*(1-tWindowOverlap);
    tE = tS + tW;
    if tE > t(end), break; end
    
    tInds = (tS <= t) & (t < tE);
    
    scores{iSegment} = computeSimilarityMatrix(dataIMU, dataCam, 1:length(dataCam.camPos), decFactor, 1, [], [], tInds);
    iSegment = iSegment+1;
end
end

