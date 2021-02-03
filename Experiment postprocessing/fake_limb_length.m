bodyPartToJointsAssoc = cell2struct([{'Head', 'nose','neck', -1}; {'Sternum', 'rShoulder','lShoulder', 0.5}; {'Pelvis', 'rHip','lHip', 0.5}; {'L_UpArm', 'lShoulder','lElbow', 0.4}; {'R_UpArm', 'rShoulder','rElbow', 0.4}; {'L_LowArm', 'lElbow','lWrist', 0.8}; {'R_LowArm', 'rElbow','rWrist', 0.8}; {'L_UpLeg', 'lHip','lKnee', 0.5}; {'R_UpLeg', 'rHip','rKnee', 0.5}; {'L_LowLeg', 'lKnee','lAnkle', 0.5}; {'R_LowLeg', 'rKnee','rAnkle', 0.5}; {'L_Foot', 'lAnkle','lSmallToe', 0.5}; {'R_Foot', 'rAnkle','rSmallToe', 0.5}], {'bodyPart', 'joint1', 'joint2', 'alpha'}, 2);
vicon_parts = [{'Head','Head'}; {'Sternum','Spine3'}; {'Pelvis','Hips'}; {'L_UpArm','LeftArm'}; {'R_UpArm', 'RightArm'}; {'L_LowArm', 'LeftForeArm'};{'R_LowArm','RightForeArm'}; {'L_UpLeg','LeftUpLeg'}; {'R_UpLeg','RightUpLeg'}; {'L_LowLeg','LeftLeg'};{'R_LowLeg','RightLeg'};{'L_Foot',	'LeftFoot'}; {'R_Foot',	'RightFoot'}];
imuToViconAssoc = struct();
for ii = 1:13
    imuToViconAssoc.(vicon_parts{ii,1}) = vicon_parts{ii,2};
end

vicon_pos = readViconPos('s1_acting/gt_skel_gbl_pos.txt');
vicon_ori = readViconOrientation('s1_acting/gt_skel_gbl_ori.txt');
vicon_calib = readViconCalib('s1_acting/s1_acting1_calib_imu_ref.txt','s1_acting/s1_acting1_calib_imu_bone.txt');

gt_dataCams = dataCams;
person_idx = 1;
n_parts = length(bodyPartToJointsAssoc);
for ii=1:n_parts
    bp_info = bodyPartToJointsAssoc(ii);
    imu_name = bp_info.bodyPart;
    j1_name = bp_info.joint1;
    j2_name = bp_info.joint2;
    for cam = 1:8
        vicon_name = imuToViconAssoc.(imu_name);
        gt_pos = vicon_pos.(vicon_name);
        world_to_cam = [dataCams(cam).params.cam.intrinsicMat [0 ; 0 ; 0]]*dataCams(cam).params.cam.extrinsicMat;
        n_frames = length(gt_pos);

        % make up a limb of length 0.25m and see what the camera coords
        % would be for the joints
        % this is needed because the algorithm is taking the normal to the
        % image plane of the line in the image
        limb_length = 0.25;
        device_orient = dataIMU.(imu_name).quat;
        [~,x,y,z] = parts(device_orient);
        limb_dir = quaternion([zeros(n_frames,1) x y z]./sqrt(sum([x y z].^2, 2)));
        
        % to make the joints, we start from the imu/limb location and make
        % sure the alpha of j1 to j2 is kept
        j1_q = limb_dir .* quaternion([0 0 0 limb_length*(bp_info.alpha-1)]).* conj(limb_dir);
        [~,x,y,z] = parts(j1_q);
        j1_vicon = [x y z] + gt_pos;
        j2_q = limb_dir .* quaternion([0 0 0 limb_length*bp_info.alpha]).* conj(limb_dir);
        [~,x,y,z] = parts(j2_q);
        j2_vicon = [x y z] + gt_pos;
        
        gt_dataCams(cam).camPos(1).(j1_name).pos3D = j1_vicon;
        gt_dataCams(cam).camPos(1).(j2_name).pos3D = j2_vicon;
        
        j1_hom = [j1_vicon'; ones(1, n_frames)];
        j2_hom = [j2_vicon'; ones(1, n_frames)];
        
        j1_pos_hom = world_to_cam * j1_hom;
        j1_pos_hom = j1_pos_hom./(j1_pos_hom(3,:)); % normalize homogenous coordinates
        j2_pos_hom = world_to_cam * j2_hom;
        j2_pos_hom = j2_pos_hom./(j2_pos_hom(3,:)); % normalize homogenous coordinates
        
        gt_j1 = j1_pos_hom(1:2,:)';
        gt_j2 = j2_pos_hom(1:2,:)';

        gt_dataCams(cam).camPos(1).(j1_name).pos2D = gt_j1;
        gt_dataCams(cam).camPos(1).(j2_name).pos2D = gt_j2;
    end
end
