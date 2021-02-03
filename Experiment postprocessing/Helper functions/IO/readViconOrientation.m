function raw_data = readViconOrientation(filename)
% Gets the Vicon ground truth orientation quaternions
% TODO: use the calibration files to rotate to imu frame?
    f = fopen(filename);  % Open the file
	
	% Parse first line: body part names
	bp_names = strsplit(fgetl(f), '\t');
    n_parts = length(bp_names);
    raw_data =struct();
	
    frame_num = 1;
	% Read all frames
	while ~feof(f)
        next_line = fgetl(f);
        if ~ischar(next_line), break; end
        pos_chunks = strsplit(next_line, '\t');
        for ii = 1:n_parts-1
            body_part = bp_names{ii};
            q_raw = str2num(pos_chunks{ii});
            q = quaternion([q_raw(4) q_raw(1) q_raw(2) q_raw(3)]);
            if frame_num == 1
                raw_data.(body_part) = q;
            else
                raw_data.(body_part)(frame_num,:) = q;
            end
        end
        frame_num = frame_num + 1;
    end
    
	
	fclose(f);
end
