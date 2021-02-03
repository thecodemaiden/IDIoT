% process the windows that are all defined
% 1) for detected-only windows of each part (some parts obscured)
% 2) for detected-only windows of whole body (all parts detected by
% openpose)

function [processed, conf] = processFullWindows(dataIMU, dataCams, personIds, tW, tWindowOverlap, frameIdxs)


n_cams = length(dataCams);
if nargin < 3|| isempty(personIds), personIds = 1; end
if nargin < 4 || isempty(tW), tW = 20; end
if nargin < 5 || isempty(tWindowOverlap), tWindowOverlap = 0.5; end


dt = 1/60;

    max_frames = length(dataCams(1).camPos(1).rWrist.pos2D);
    if nargin < 6 || isempty(frameIdxs), frameIdxs = 1:max_frames; end
    n_frames = length(frameIdxs);

    outputLen = floor((n_frames - tW/dt)/((1-tWindowOverlap)*tW/dt)) + 1;
processed = cell(n_cams,outputLen);
conf = cell(n_cams, 1);
n_to_process = outputLen*n_cams;


w = waitbar(0, 'Computing similarity matrix for multiple camera angles...');
% TODO: make the conf a 8x1 cell matrix, with 13xnwin entries inside
win_time = frameIdxs./dataCams(1).fps;
t = (0:max_frames-1)./dataCams(1).fps;

for iCam = 1:n_cams %1:length(dataPreProc.dataCams)
    tS = win_time(1);
    iSegment = 1;
    while true
        tE = tS + tW;

        tInds = (tS <= t) & (t < tE);
        
        [aa,bb] = doShit(dataIMU, dataCams(iCam), personIds, 1, [], [], tInds);
        processed{iCam, iSegment} = aa;
        conf{iCam}(:,iSegment) = bb;
        waitbar((iSegment+(iCam-1)*outputLen)/n_to_process, w, sprintf('Processing cam %d/%d, window %d/%d', iCam, length(n_cams), iSegment, outputLen));
        iSegment = iSegment+1;
        tS = tS+tW*(1-tWindowOverlap);
        if tE > t(end), break; end

    end
end
close(w);
% Save temp results
fprintf('Done\n');
end


function [scoreIMUtoCamBodyPart, windowConfidence] = doShit(dataIMU, dataCam, personIDs, decBy, indsIMU, indsBodyJointPairs, tInds)
% return nan for any body part that didn't have a fully non-nan window

if nargin<3 || isempty(personIDs), personIDs = 1; end
if nargin<4 || isempty(decBy) || decBy < 1, decBy = 1; end

% todo: head is left ear - right ear?
bodyPartToJointsAssoc = cell2struct([{'Head', 'nose','neck', -1}; {'Sternum', 'rShoulder','lShoulder', 0.5}; {'Pelvis', 'rHip','lHip', 0.5}; {'L_UpArm', 'lShoulder','lElbow', 0.4}; {'R_UpArm', 'rShoulder','rElbow', 0.4}; {'L_LowArm', 'lElbow','lWrist', 0.8}; {'R_LowArm', 'rElbow','rWrist', 0.8}; {'L_UpLeg', 'lHip','lKnee', 0.5}; {'R_UpLeg', 'rHip','rKnee', 0.5}; {'L_LowLeg', 'lKnee','lAnkle', 0.5}; {'R_LowLeg', 'rKnee','rAnkle', 0.5}; {'L_Foot', 'lAnkle','lSmallToe', 0.5}; {'R_Foot', 'rAnkle','rSmallToe', 0.5}], {'bodyPart', 'joint1', 'joint2', 'alpha'}, 2);
namesIMUs = setdiff(fieldnames(dataIMU), 'params', 'stable');  % Take all the IMUs (remove the field 'params'). 'stable' keeps the original ordering of fieldnames(dataIMU).
if nargin<5 || isempty(indsIMU), indsIMU = 1:length(namesIMUs); end
if nargin<6 || isempty(indsBodyJointPairs), indsBodyJointPairs = 1:length(bodyPartToJointsAssoc); end
if nargin<7 || isempty(tInds), tInds = 1:length(dataIMU.(namesIMUs{1}).quat); end

decBy = ceil(decBy);
scoreIMUtoCamBodyPart = ones(length(indsIMU), length(personIDs)*length(indsBodyJointPairs));
windowConfidence = zeros(1, length(personIDs)*length(indsBodyJointPairs));
for iIMU = 1:length(indsIMU)
    nameIMU = namesIMUs{indsIMU(iIMU)};
    
    for iPerson = 1:length(personIDs)
        personID = personIDs(iPerson);
        
        for iBodyJointPair = 1:length(indsBodyJointPairs)
            jointAssoc = bodyPartToJointsAssoc(indsBodyJointPairs(iBodyJointPair));
            iInd = (iPerson-1)*length(indsBodyJointPairs) + iBodyJointPair;
            
            orientationIMU = dataIMU.(nameIMU).quat(tInds,:);
            gyroIMU = dataIMU.(nameIMU).gyro(tInds,:);
            posCamJoints2D = [dataCam.camPos(personID).(jointAssoc.joint1).pos2D(tInds,:) dataCam.camPos(personID).(jointAssoc.joint2).pos2D(tInds,:)];            
            
            if decBy > 1
                % resample
                posCamJoints2D = posCamJoints2D(1:decBy:end,:);
                orientationIMU = orientationIMU(1:decBy:end,:);
                gyroIMU = gyroIMU(1:decBy:end,:);
            end
            validInds = all(~isnan(posCamJoints2D), 2);
            
            % convert the gyro readings to quaternion form
            windowConfidence(iInd) = sum(validInds)/length(validInds);
            if sum(validInds) > 1
                % don't bother calling if there aren't 2 or more
                % measurements in this window
                scoreIMUtoCamBodyPart(iIMU, iInd) = find_shift_and_alignment(posCamJoints2D, orientationIMU, gyroIMU, dataCam.params.cam.intrinsicMat, 0);
            end
        end
    end
end
end
