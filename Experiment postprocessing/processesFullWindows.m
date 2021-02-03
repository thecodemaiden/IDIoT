
function [processed, conf] = processesFullWindows(dataIMU, dataCams, fW, fWindowOverlap, includeMask,  personIDs)


n_cams = length(dataCams);
if nargin < 6|| isempty(personIDs), personIDs = 1; end
if nargin < 3 || isempty(fW), fW = 1200; end
if nargin < 4 || isempty(fWindowOverlap), fWindowOverlap = 0.5; end

vector_mask = isvector(includeMask);
use_mask = true;
if nargin < 5 || isempty(includeMask), use_mask = false; end
    
max_frames = length(dataCams(1).camPos(1).rWrist.pos2D);

outputLen = floor((max_frames - fW)/((1-fWindowOverlap)*fW)) + 1;
processed = cell(n_cams,outputLen);
conf = cell(n_cams, 1);


w = waitbar(0, 'Computing similarity matrix for multiple camera angles...');

for iCam = 1:n_cams %1:length(dataPreProc.dataCams)
    fS = 1;
    iSegment = 1;

    while true
        fE = fS + fW - 1;
        if fE > max_frames, break; end
        window_mask = false(max_frames,1);
        window_mask(fS:fE) = true;
        % todo: head is left ear - right ear?
        bodyPartToJointsAssoc = cell2struct([{'Head', 'nose','neck', -1}; {'Sternum', 'rShoulder','lShoulder', 0.5}; {'Pelvis', 'rHip','lHip', 0.5}; {'L_UpArm', 'lShoulder','lElbow', 0.4}; {'R_UpArm', 'rShoulder','rElbow', 0.4}; {'L_LowArm', 'lElbow','lWrist', 0.8}; {'R_LowArm', 'rElbow','rWrist', 0.8}; {'L_UpLeg', 'lHip','lKnee', 0.5}; {'R_UpLeg', 'rHip','rKnee', 0.5}; {'L_LowLeg', 'lKnee','lAnkle', 0.5}; {'R_LowLeg', 'rKnee','rAnkle', 0.5}; {'L_Foot', 'lAnkle','lSmallToe', 0.5}; {'R_Foot', 'rAnkle','rSmallToe', 0.5}], {'bodyPart', 'joint1', 'joint2', 'alpha'}, 2);
        namesIMUs = setdiff(fieldnames(dataIMU), 'params', 'stable');  % Take all the IMUs (remove the field 'params'). 'stable' keeps the original ordering of fieldnames(dataIMU).
        indsIMU = 1:length(namesIMUs);
        indsBodyJointPairs = 1:length(bodyPartToJointsAssoc);
        
        dataCam = dataCams(iCam);
        
        decBy = 1;
        %decBy = ceil(decBy);
        scoreIMUtoCamBodyPart = ones(length(indsIMU), length(personIDs)*length(indsBodyJointPairs));
        windowConfidence = zeros(1, length(personIDs)*length(indsBodyJointPairs));
        for iIMU = 1:length(indsIMU)
            nameIMU = namesIMUs{indsIMU(iIMU)};
            frame_mask = true(max_frames,1);
            if use_mask
                if vector_mask
                    frame_mask = frame_mask & includeMask;
                else
                    frame_mask = frame_mask & includeMask.(nameIMU);
                end
            end
%             if use_rotation
%                 rot_err = rotation_error.(nameIMU).rot_err;
%                 frame_mask = frame_mask & rot_err >= max_rot_err;
%             end
%             if use_position
%                 pos_err = position_error.(nameIMU)(iCam).pos_err;
%                 frame_mask = frame_mask & pos_err >= max_pos_err;
%             end
            tMask = window_mask & frame_mask;
            for iPerson = 1:length(personIDs)
                personID = personIDs(iPerson);
                
                for iBodyJointPair = 1:length(indsBodyJointPairs)
                    jointAssoc = bodyPartToJointsAssoc(indsBodyJointPairs(iBodyJointPair));
                    iInd = (iPerson-1)*length(indsBodyJointPairs) + iBodyJointPair;
                    
                    orientationIMU = dataIMU.(nameIMU).quat(tMask,:);
                    gyroIMU = dataIMU.(nameIMU).gyro(tMask,:);
                    posCamJoints2D = [dataCam.camPos(personID).(jointAssoc.joint1).pos2D(tMask,:) dataCam.camPos(personID).(jointAssoc.joint2).pos2D(tMask,:)];
                    
                    if decBy > 1
                        % resample
                        posCamJoints2D = posCamJoints2D(1:decBy:end,:);
                        orientationIMU = orientationIMU(1:decBy:end,:);
                        gyroIMU = gyroIMU(1:decBy:end,:);
                    end
                    validInds = all(~isnan(posCamJoints2D), 2);
                    
                    windowConfidence(iInd) = sum(validInds)/length(validInds);
                    if sum(validInds) > 1
                        % don't bother calling if there aren't 2 or more
                        % measurements in this window
                        scoreIMUtoCamBodyPart(iIMU, iInd) = find_shift_and_alignment(posCamJoints2D, orientationIMU, gyroIMU, dataCam.params.cam.intrinsicMat, 0);
                    end
                end
            end
            waitbar(iIMU/length(indsIMU), w, sprintf('Processing cam %d/%d, window %d/%d', iCam, length(n_cams), iSegment, outputLen));

        end
        
        processed{iCam, iSegment} = scoreIMUtoCamBodyPart;
        conf{iCam}(:,iSegment) = windowConfidence;
        iSegment = iSegment+1;
        fS = fS+fW*(1-fWindowOverlap);
        
    end
    close(w);
    % Save temp results
    fprintf('Done\n');
end

