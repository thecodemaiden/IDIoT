% for each camera, eliminate the body parts whose max score is below a
% threshold

good_enough = 0.18;
nCams =8;
nWin = 5;
assnByCam = cell(1,nCams);
limb_info = cell2struct([{'Head', 'nose','neck', -1}; {'Sternum', 'rShoulder','lShoulder', 0.5}; {'Pelvis', 'rHip','lHip', 0.5}; {'L_UpArm', 'lShoulder','lElbow', 0.4}; {'R_UpArm', 'rShoulder','rElbow', 0.4}; {'L_LowArm', 'lElbow','lWrist', 0.8}; {'R_LowArm', 'rElbow','rWrist', 0.8}; {'L_UpLeg', 'lHip','lKnee', 0.5}; {'R_UpLeg', 'rHip','rKnee', 0.5}; {'L_LowLeg', 'lKnee','lAnkle', 0.5}; {'R_LowLeg', 'rKnee','rAnkle', 0.5}; {'L_Foot', 'lAnkle','lSmallToe', 0.5}; {'R_Foot', 'rAnkle','rSmallToe', 0.5}], {'bodyPart', 'joint1', 'joint2', 'alpha'}, 2);
nLimbs = length(limb_info);
for ii=1:nCams
    assigned_imu2pose = nan(nLimbs,1);% for each imu, what pose might be assigned
    for jj = 1:nWin
        costToAssign =  scores_tw_20{ii}{jj}(1:nLimbs,1:nLimbs);
        %zero out the cost of the already assigned IMU-pose pairs
        fixed_imus = ~isnan(assigned_imu2pose);
        
        for kk=find(fixed_imus)'
            costToAssign(kk, assigned_imu2pose(kk)) = 0;
        end
      
        [imu2poseIdx] = assign2D(costToAssign);
        
        % lock in the pair if it is certain enough
        can_lock_imu = (range(costToAssign,2) > good_enough) & ~fixed_imus;
        assigned_imu2pose(can_lock_imu) = imu2poseIdx(can_lock_imu);
        current_a = assigned_imu2pose;
        current_a(~fixed_imus) = imu2poseIdx(~fixed_imus);
        assnByCam{ii}(:,jj) = current_a;
    end
end
        
    



% % for each camera, eliminate the body parts whose max score is below a
% % threshold
% 
% good_enough = 0.15;
% nCams =8;
% nWin = 5;
% assnByCam = cell(1,nCams);
% limb_info = cell2struct([{'Head', 'nose','neck', -1}; {'Sternum', 'rShoulder','lShoulder', 0.5}; {'Pelvis', 'rHip','lHip', 0.5}; {'L_UpArm', 'lShoulder','lElbow', 0.4}; {'R_UpArm', 'rShoulder','rElbow', 0.4}; {'L_LowArm', 'lElbow','lWrist', 0.8}; {'R_LowArm', 'rElbow','rWrist', 0.8}; {'L_UpLeg', 'lHip','lKnee', 0.5}; {'R_UpLeg', 'rHip','rKnee', 0.5}; {'L_LowLeg', 'lKnee','lAnkle', 0.5}; {'R_LowLeg', 'rKnee','rAnkle', 0.5}; {'L_Foot', 'lAnkle','lSmallToe', 0.5}; {'R_Foot', 'rAnkle','rSmallToe', 0.5}], {'bodyPart', 'joint1', 'joint2', 'alpha'}, 2);
% nLimbs = length(limb_info);
% for ii=1:nCams
%     assigned_pose2imu = nan(nLimbs,1);% for each pose component, what imu it might be
%     assigned_imu2pose = nan(nLimbs,1);% for each imu, what pose might be assigned
%     for jj = 1:nWin
%         all_scores =  scores_tw_20{ii}{jj};
%         unassigned_pose = find(isnan(assigned_pose2imu));
%         unassigned_imu = find(isnan(assigned_imu2pose));
%         nToAssign = length(unassigned_pose);
%         costToAssign = all_scores(unassigned_imu, unassigned_pose);
%         [imu2poseIdx, pose2imuIdx] = assign2D(costToAssign);
%         match_imu = unassigned_imu(pose2imuIdx);
%         match_pose = unassigned_pose(imu2poseIdx);
%         scoresByImu = all_scores(unassigned_imu, 1:nLimbs);
%         %for ii=1:nToAssign
%         %    assignCosts(ii) = all_scores(match_imu(ii), match_pose(ii));
%         %end
%         can_lock_imu = range(scoresByImu,2) > good_enough;
%         new_imu_a = unassigned_imu(can_lock_imu);
%         lock_pose_idx = unassigned_pose(imu2poseIdx(can_lock_imu));
%         assigned_imu2pose(new_imu_a) = lock_pose_idx;
%         assigned_pose2imu(lock_pose_idx) = new_imu_a;
%         
%         current_a = assigned_imu2pose;
%         current_a(match_imu) = match_pose;
%         assnByCam{ii}(:,jj) = current_a;
%     end
% end
%         
%     