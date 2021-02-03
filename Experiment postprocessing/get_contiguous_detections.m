% for each body part, chunk the contiguous frames where part is identified

body_parts = fieldnames(dataCams(1).camPos(1));
n_parts = length(body_parts);
n_cams = length(dataCams);
chunk_idxs = cell(n_parts,n_cams);

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