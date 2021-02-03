function raw_data = readViconPos(filename)
% Gets the Vicon ground truth position of the body parts, converted to m	
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
            p = str2num(pos_chunks{ii});
            p = p.*0.0245; % in to m
            if frame_num == 1
                raw_data.(body_part) = p;
            else
                raw_data.(body_part)(frame_num, :) = p;
            end
        end
        frame_num = frame_num + 1;
    end
    
	
	fclose(f);
end
