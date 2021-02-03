%scoreByCam - cell(nCams x nWindows)
nCams = size(scoreByCam,1);
nWindows = size(scoreByCam,2);
%imuConfidence has the percent present for each window
conf_thresh = 0.1; % minimum percent present to use the window


assignments = cell(nCams, 1);
score_mat = cell(nCams,nWindows);
filtered_assignments = cell(nCams,1);
alpha = 0.5; % adjust low pass filtering coefficient
for cc = 1:nCams
    last_score = ones(13,13);
    a = zeros(13,nWindows);
    for nn = 1:nWindows
        openpose_detected = imuConfidence{cc}(:,nn) >= conf_thresh;
        imu_idxs = find(openpose_detected);
        if isempty(imu_idxs), continue; end
        scores_to_match = scoreByCam{cc,nn}(openpose_detected,:);
        
        %update the filtered score
        update_score = last_score;
        update_score(openpose_detected,:) = scores_to_match;
        last_score = (1-alpha)*update_score + alpha*last_score;
        [matched_imus] = assign2D(scores_to_match);
        a(openpose_detected,nn) = matched_imus;

        score_mat{cc,nn} = last_score;
        [filt_match] = assign2D(last_score);
        filtered_assignments{cc}(:,nn) = filt_match;
    end
    assignments{cc} = a;
end