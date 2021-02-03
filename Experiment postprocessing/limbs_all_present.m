% for each body part, chunk the contiguous frames where part is identified

body_parts = fieldnames(dataCams(1).camPos(1));
n_parts = length(body_parts);
n_cams = length(dataCams);
chunk_idxs = cell(n_parts,n_cams);


% when the two body parts are both ddetected
bp_temp = cell2struct([{'Head', 'nose','neck', -1}; {'Sternum', 'rShoulder','lShoulder', 0.5}; {'Pelvis', 'rHip','lHip', 0.5}; {'L_UpArm', 'lShoulder','lElbow', 0.4}; {'R_UpArm', 'rShoulder','rElbow', 0.4}; {'L_LowArm', 'lElbow','lWrist', 0.8}; {'R_LowArm', 'rElbow','rWrist', 0.8}; {'L_UpLeg', 'lHip','lKnee', 0.5}; {'R_UpLeg', 'rHip','rKnee', 0.5}; {'L_LowLeg', 'lKnee','lAnkle', 0.5}; {'R_LowLeg', 'rKnee','rAnkle', 0.5}; {'L_Foot', 'lAnkle','lSmallToe', 0.5}; {'R_Foot', 'rAnkle','rSmallToe', 0.5};], {'bodyPart', 'joint1', 'joint2', 'alpha'}, 2);
%bp_temp = cell2struct([{'Head', 'nose','neck', -1}; {'Head2','lEar','rEar',0.5}; {'Sternum', 'rShoulder','lShoulder', 0.5}; {'Pelvis', 'rHip','lHip', 0.5}; {'L_UpArm', 'lShoulder','lElbow', 0.4}; {'R_UpArm', 'rShoulder','rElbow', 0.4}; {'L_LowArm', 'lElbow','lWrist', 0.8}; {'R_LowArm', 'rElbow','rWrist', 0.8}; {'L_UpLeg', 'lHip','lKnee', 0.5}; {'R_UpLeg', 'rHip','rKnee', 0.5}; {'L_LowLeg', 'lKnee','lAnkle', 0.5}; {'R_LowLeg', 'rKnee','rAnkle', 0.5}; {'L_Foot', 'lAnkle','lSmallToe', 0.5}; {'L_Foot2', 'lHeel', 'lBigToe', 0.5}; {'R_Foot', 'rAnkle','rSmallToe', 0.5}; {'R_Foot2', 'rHeel', 'rBigToe', 0.5}], {'bodyPart', 'joint1', 'joint2', 'alpha'}, 2);
n_parts = length(bp_temp);
display_names = {bp_temp.bodyPart};
n_cams = 8;
window_sizes = zeros(1,n_cams);
window_starts = zeros(1,n_cams);
for cc = 1:n_cams
    n_frames = length(dataCams(cc).camPos(1).nose.pos2D(:,1));
    any_missing = false(n_frames,1);
    for ii=1:n_parts
            j1_missing = isnan(dataCams(cc).camPos(1).(bp_temp(ii).joint1).pos2D(:,1));
            j2_missing = isnan(dataCams(cc).camPos(1).(bp_temp(ii).joint2).pos2D(:,1));

            any_missing = any_missing | (j1_missing | j2_missing);
    end
            detect_change = diff(any_missing);
            start_is_missing = (any_missing(1));
            end_idx = length(any_missing);
            nan_to_real = find(detect_change == -1);
            real_to_nan = find(detect_change == 1);
            % Do I need to check if it was nan at the start?
            minlen = min(length(nan_to_real),length(real_to_nan));
    
            if minlen == 0
                % there were no changes in the whole file
                if start_is_missing
                    ranges = 0;
                else
                    ranges = end_idx;
                end
            else
                if ~start_is_missing
                    real_to_nan = [real_to_nan; end_idx];
                    nan_to_real = [0; nan_to_real];
                end
                ranges = real_to_nan(1:minlen) - nan_to_real(1:minlen);

            end
            [mv, mi] = max(ranges);
            window_sizes(cc) = mv;
            if minlen > 0
                window_starts(cc) = nan_to_real(mi)+1;
            end
end


for cc = 1:n_cams
    for nn=1:n_parts
        bp = body_parts{nn};
        detect_change = diff(isnan(dataCams(cc).camPos(1).(bp).pos2D(:,1)));
        start_is_nan = isnan(dataCams(cc).camPos(1).(bp).pos2D(1));
        end_idx = length(detect_change);
        nan_to_real = find(detect_change == -1);
        real_to_nan = find(detect_change == 1);
        
        % I need to check if it was nan at the start
        minlen = min(length(nan_to_real),length(real_to_nan));
        
        if minlen == 0
            % there were no changes in the whole file
            if start_is_nan
                ranges = 0;
            else
                ranges = end_idx;
            end
        else
            if ~start_is_nan
                real_to_nan = [real_to_nan; end_idx];
                nan_to_real = [0; nan_to_real];
            end
            ranges = real_to_nan(1:minlen) - nan_to_real(1:minlen);
            
        end
        
        
        chunk_idxs{nn,cc} = ranges;
    end
end

% head accuracy is bad because nose is bad (sort by max in chunk_idx)