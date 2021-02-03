% mid_part = 'Sternum';
% limb_part = 'L_UpLeg';
% n_frames = 2000;%length(dataIMU.(limb_part).gyro);
% dt = 1/60;
% t_idxs = 1:n_frames; % 100 frames = 1.6667s
% gyro_data_limb = dataIMU.(limb_part).gyro(t_idxs,:);
% orient_limb = dataIMU.(limb_part).quat(t_idxs,:);
%
% gyro_data_mid = dataIMU.(mid_part).gyro(t_idxs,:);
% orient_mid = dataIMU.(mid_part).quat(t_idxs,:);
%
% align_scores_correct = [0 0]; % [limb mid]
%
% v = VideoWriter('matching_limb.avi');
% start_idx = 1;
% win_len = 10;
% h = figure(1);
% ff=1;
% open(v);
% while start_idx < n_frames - win_len
%     %angle_gyro_head = sum(gyro_data_head(start_idx:start_idx+win_len,:));
%     gyro_quat_head =  accum_gyro(gyro_data_mid(start_idx:start_idx+win_len,:), dt); %gyroToQuaternion(angle_gyro_head, win_len/60);
%
%     %angle_gyro_arm = sum(gyro_data_arm(start_idx:start_idx+win_len,:));
%     gyro_quat_arm = accum_gyro(gyro_data_limb(start_idx:start_idx+win_len,:), dt);%gyroToQuaternion(angle_gyro_arm, win_len/60);
%
%     limb_win_quat = orient_limb(start_idx+win_len)*conj(orient_limb(start_idx));
%     mid_win_quat = orient_mid(start_idx+win_len)*conj(orient_mid(start_idx));
%
%     limb_origin = quaternion([0 0 0 1]); % along z axis
%
%     rot_limb_quat = limb_win_quat*limb_origin*conj(limb_win_quat);
%     rot_mid_quat = mid_win_quat*limb_origin*conj(mid_win_quat);
%
%     rot_limb_gyro = gyro_quat_arm*limb_origin*conj(gyro_quat_arm);
%     rot_mid_gyro = gyro_quat_head*limb_origin*conj(gyro_quat_head);
%
%     [~, ax_ao] = ang_ax(rot_limb_quat);
%     [~, ax_ho] = ang_ax(rot_mid_quat);
%
%     [~, ax_ag] = ang_ax(rot_limb_gyro);
%     [~, ax_hg] = ang_ax(rot_mid_gyro);
%
%     align_scores_correct(ff,1) = dot(ax_ao, ax_ag);
%     align_scores_correct(ff,2) = dot(ax_ho, ax_hg);
%
%     subplot(2,2,1)
%     quiver3(0,0,0, ax_ho(1), ax_ho(2), ax_ho(3));
%     xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
%     title([mid_part '-quat']);
%     subplot(2,2,2)
%     quiver3(0,0,0, ax_hg(1), ax_hg(2), ax_hg(3));
%     xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
%     title([mid_part '-gyro']);
%         subplot(2,2,3)
%     quiver3(0,0,0, ax_ao(1), ax_ao(2), ax_ao(3));
%     xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
%     title([limb_part '-quat']);
%     subplot(2,2,4)
%     quiver3(0,0,0, ax_ag(1), ax_ag(2), ax_ag(3));
%     xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
%     title([limb_part '-gyro']);
%     pause(0.05);
%     start_idx = start_idx+2;
%     ff=ff+1;
%     fr = getframe(h);
%     writeVideo(v,fr);
% end
% close(v);
%
% function y = accum_gyro(readings, dt)
%
% converted = gyroToQuaternion(readings, dt);
% y = quaternion([1 0 0 0 ]);
% for ii=1:length(readings)
%     y = y*converted(ii);
% end
%
%
% end


mid_part = 'Sternum';
limb_part = 'L_UpLeg';
n_frames = length(dataIMU.(limb_part).gyro);
dt = 1/60;
t_idxs = 1:n_frames; % 100 frames = 1.6667s

start_idx = 1;
win_len = 10;
win_slide = 1;

body_parts = fieldnames(dataIMU);
n_parts = length(body_parts);
align_scores = zeros(0,n_parts);
ff = 1;
while start_idx < n_frames - win_len
    for jj = 1:n_parts
        part_name = body_parts{jj};
        gyro_data = dataIMU.(part_name).gyro(start_idx:start_idx+win_len,:);
        orient_filt = dataIMU.(part_name).quat(start_idx:start_idx+win_len,:);
        
        gyro_quat =  accum_gyro(gyro_data, dt);
        
        win_quat = dataIMU.(part_name).quat(start_idx+win_len)*conj(dataIMU.(part_name).quat(start_idx));
        
        limb_origin = quaternion([0 0 0 1]); % along z axis
        
        rot_quat = win_quat*limb_origin*conj(win_quat);
        
        rot_gyro = gyro_quat*limb_origin*conj(gyro_quat);
        
        [~, ax_q] = ang_ax(rot_quat);
        
        [~, ax_g] = ang_ax(rot_gyro);
        
        align_scores(ff,jj) = dot(ax_q, ax_g);
    end
    start_idx = start_idx+win_slide;
    ff=ff+1;   
end

function y = accum_gyro(readings, dt)

converted = gyroToQuaternion(readings, dt);
y = quaternion([1 0 0 0 ]);
for ii=1:length(readings)
    y = y*converted(ii);
end


end

% features:
% mean alignment between gyro and orient change in window
% accel total/mean over window
% moving average/std of alignment

% so: - take a window of ~60 frames (1s)
%     - do a slide by ~20 frames
%     - calculate the mean/stds for the features
%     - train kMeans?

