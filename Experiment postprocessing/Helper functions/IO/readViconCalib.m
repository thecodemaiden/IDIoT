function calib_data = readViconCalib(ref_file, bone_file)
% Gets the Vicon ground truth position of the body parts, converted to m	
    bf = fopen(bone_file);  % Open the file
    rf = fopen(ref_file);  % Open the file
	
    n_parts = str2double(fgetl(bf));
    n2 = str2double(fgetl(rf));
    assert(n_parts == n2, 'Mismatch in body parts');

    calib_data =struct();
	
   for ii=1:n_parts
       % read data from both files, make sure the parts are in the same
       % order
       bl = strsplit(fgetl(bf),'\t');
       rl = strsplit(fgetl(rf),'\t');
       
       bp_name = bl{1};
       bp2 = rl{1};

       assert(all(bp_name == bp2), 'Mismatch in body parts');
       
       bone_quat = str2num(bl{2:end});
       ref_quat = str2num(rl{2:end});
       
       s = struct();
       s.q_bone = quaternion(bone_quat([4 1 2 3]));
       s.q_ref = quaternion(ref_quat([4 1 2 3]));
       
       calib_data.(bp_name) = s;
   end
	fclose(rf);
	fclose(bf);
end
