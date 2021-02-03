
% features:
% mean alignment between gyro and orient change in window
% accel total/mean over window
% moving average/std of alignment

% so: - take a window of ~20 frames (1/3s)
%     - do a slide by ~5 frames
%     - calculate the mean/stds for the features
%     - train kMeans?


n_frames = length(dataIMU.Head.gyro);
dt = 1/60;
t_idxs = 1:n_frames; 

start_idx = 1;
win_len = 20;
win_slide = 5;

body_parts = fieldnames(dataIMU);
n_parts = length(body_parts);
align_scores = zeros(0,n_parts);
accel_vals = zeros(0, n_parts);

ff = 1;
while start_idx < n_frames - win_len
    for jj = 1:n_parts
        part_name = body_parts{jj};
        gyro_data = dataIMU.(part_name).gyro(start_idx:start_idx+win_len,:);
        accel_data = dataIMU.(part_name).accel(start_idx:start_idx+win_len,:);
        %orient_filt = dataIMU.(part_name).quat(start_idx:start_idx+win_len,:);
        
        gyro_quat =  accum_gyro(gyro_data, dt);
        
        win_quat = dataIMU.(part_name).quat(start_idx+win_len)*conj(dataIMU.(part_name).quat(start_idx));
        
        limb_origin = quaternion([0 0 0 1]); % along z axis
        
        rot_quat = win_quat*limb_origin*conj(win_quat);
        
        rot_gyro = gyro_quat*limb_origin*conj(gyro_quat);
        
        [~, ax_q] = ang_ax(rot_quat);
        
        [~, ax_g] = ang_ax(rot_gyro);
        
        align_scores(ff,jj) = dot(ax_q, ax_g);
        accel_vals(ff,jj) = mean(sqrt(sum(accel_data.^2,2)));
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
