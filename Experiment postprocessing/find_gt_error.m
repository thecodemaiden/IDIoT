bodyPartToJointsAssoc = cell2struct([{'Head', 'nose','neck', -1}; {'Sternum', 'rShoulder','lShoulder', 0.5}; {'Pelvis', 'rHip','lHip', 0.5}; {'L_UpArm', 'lShoulder','lElbow', 0.4}; {'R_UpArm', 'rShoulder','rElbow', 0.4}; {'L_LowArm', 'lElbow','lWrist', 0.8}; {'R_LowArm', 'rElbow','rWrist', 0.8}; {'L_UpLeg', 'lHip','lKnee', 0.5}; {'R_UpLeg', 'rHip','rKnee', 0.5}; {'L_LowLeg', 'lKnee','lAnkle', 0.5}; {'R_LowLeg', 'rKnee','rAnkle', 0.5}; {'L_Foot', 'lAnkle','lSmallToe', 0.5}; {'R_Foot', 'rAnkle','rSmallToe', 0.5}], {'bodyPart', 'joint1', 'joint2', 'alpha'}, 2);
vicon_parts = [{'Head','Head'}; {'Sternum','Spine3'}; {'Pelvis','Hips'}; {'L_UpArm','LeftArm'}; {'R_UpArm', 'RightArm'}; {'L_LowArm', 'LeftForeArm'};{'R_LowArm','RightForeArm'}; {'L_UpLeg','LeftUpLeg'}; {'R_UpLeg','RightUpLeg'}; {'L_LowLeg','LeftLeg'};{'R_LowLeg','RightLeg'};{'L_Foot',	'LeftFoot'}; {'R_Foot',	'RightFoot'}];
imuToViconAssoc = struct();
for ii = 1:13
    imuToViconAssoc.(vicon_parts{ii,1}) = vicon_parts{ii,2};
end

vicon_pos = readViconPos('s1_acting/gt_skel_gbl_pos.txt');
vicon_ori = readViconOrientation('s1_acting/gt_skel_gbl_ori.txt');
vicon_calib = readViconCalib('s1_acting/s1_acting1_calib_imu_ref.txt','s1_acting/s1_acting1_calib_imu_bone.txt');

compare_ori = struct();
for cf = fieldnames(imuToViconAssoc)'
    imu_name = cf{1};
    vicon_name = imuToViconAssoc.(imu_name);
    gt_pos = vicon_ori.(vicon_name);
    imu_ori = dataIMU.(imu_name).quat;
    rot_ref = vicon_calib.(imu_name).q_ref;
    rot_bone = vicon_calib.(imu_name).q_bone;
    gt_imu_ori = conj(rot_ref).*gt_pos.*(rot_bone);
    rot_error = conj(imu_ori).*gt_imu_ori;
    % there is (rarely) a norm slightly > 1, so normalize
    rot_error = 1./norm(rot_error).*rot_error;
    s = struct('device',imu_ori, 'vicon', gt_imu_ori, 'rot_err', wrapToPi(acos(parts(rot_error))*2));
    compare_ori.(imu_name) = s;
end

%calculate alpha scores based on seen vs felt acc? use to match parts?

compare_pos = struct();
person_idx = 1;
n_parts = length(bodyPartToJointsAssoc);
for ii=1:n_parts
    bp_info = bodyPartToJointsAssoc(ii);
    imu_name = bp_info.bodyPart;
    for cam = 1:8
        vicon_name = imuToViconAssoc.(imu_name);
        gt_pos = vicon_pos.(vicon_name);
        world_to_cam = [dataCams(cam).params.cam.intrinsicMat [0 ; 0 ; 0]]*dataCams(cam).params.cam.extrinsicMat;
        
        j1_cam = dataCams(cam).camPos(person_idx).(bp_info.joint1).pos2D;
        j2_cam = dataCams(cam).camPos(person_idx).(bp_info.joint2).pos2D;
        est_pos_cam = j1_cam + (j2_cam-j1_cam).*bp_info.alpha;
       
        n_frames = length(gt_pos);
        gt_pos_hom = [gt_pos'; ones(1, n_frames)];
        
        gt_cam_pos_hom = world_to_cam * gt_pos_hom;
        gt_cam_pos_hom = gt_cam_pos_hom./(gt_cam_pos_hom(3,:)); % normalize homogenous coordinates
        
        gt_cam_pos = gt_cam_pos_hom(1:2,:)';

        pos_error = est_pos_cam-gt_cam_pos;
        pos_error = sqrt(sum(pos_error.^2,2));
        
        n_missing = sum(isnan(pos_error))./n_frames;
        
        s = struct('j1_c',j1_cam, 'j2_c',j2_cam, 'cam_bone', est_pos_cam, 'vicon_bone', gt_cam_pos, 'pos_err', pos_error, 'missing', n_missing);
        compare_pos.(imu_name)(cam) = s;
    end
end

% to remve pairs from consideration, use scores(not_limb, is_limb) = inf
% and scores(is_limb, not_limb) = inf[