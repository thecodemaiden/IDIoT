% how to find  the quaternion in camera space
% the final angle would be equal to the rotation angle seen by the
% gyroscope if there is no acceleration(?)
dt = 1/60;
t_idxs = 2500:2600; % 100 frames = 1.6667s
gyro_data_arm = dataIMU.L_LowArm.gyro(t_idxs,:);
orient_low_arm = dataIMU.L_LowArm.quat(t_idxs,:);

gyro_data_head = dataIMU.Head.gyro(t_idxs,:);
orient_head = dataIMU.Head.quat(t_idxs,:);

rot_idxs = [1 60]; % one second of rotation
angle_gyro_head = sum(gyro_data_head(rot_idxs(1):rot_idxs(2),:));
gyro_quat_head = gyroToQuaternion(angle_gyro_head, diff(rot_idxs)+1);

angle_gyro_arm = sum(gyro_data_arm(rot_idxs(1):rot_idxs(2),:));
gyro_quat_arm = gyroToQuaternion(angle_gyro_arm, diff(rot_idxs)+1);


ang_ax = @quat_angle_axis;

% visual info: x,y coordinates of corresponding joints
%limb_info = cell2struct([{'Head', 'nose','neck', -1}; {'Sternum', 'rShoulder','lShoulder', 0.5}; {'Pelvis', 'rHip','lHip', 0.5}; {'L_UpArm', 'lShoulder','lElbow', 0.4}; {'R_UpArm', 'rShoulder','rElbow', 0.4}; {'L_LowArm', 'lElbow','lWrist', 0.8}; {'R_LowArm', 'rElbow','rWrist', 0.8}; {'L_UpLeg', 'lHip','lKnee', 0.5}; {'R_UpLeg', 'rHip','rKnee', 0.5}; {'L_LowLeg', 'lKnee','lAnkle', 0.5}; {'R_LowLeg', 'rKnee','rAnkle', 0.5}; {'L_Foot', 'lAnkle','lSmallToe', 0.5}; {'R_Foot', 'rAnkle','rSmallToe', 0.5}], {'bodyPart', 'joint1', 'joint2', 'alpha'}, 2);
joint1 = 'lElbow';
joint2 = 'lWrist';
cam_id = 4;
j1 = dataCams(cam_id).camPos(1).(joint1).pos2D(t_idxs,:);
j2 = dataCams(cam_id).camPos(1).(joint2).pos2D(t_idxs,:);
[gtD2C, est_twist, est_swing] = vision_to_orientation(orient_low_arm, j1, j2, 60, 5);

% now check what we get for all IMUs
imu_names = fieldnames(dataIMU);

imu_rot = quaternion();
z1_est = [];
z2_est = [];
for nn = 1:13
    body_part = imu_names{nn};
    orient_device = dataIMU.(body_part).quat(t_idxs,:);
    [bp_d2c, bp_twist, bp_swing, z1_est, z2_est] = vision_to_orientation(orient_device, j1, j2, 60, 5, z1_est,z2_est);
    imu_rot(nn,:) = [bp_d2c, bp_twist, bp_swing];
    display([z1_est, z2_est]);
end



function [d2cEst, cam_twist, cam_swing, z1, z2] = vision_to_orientation(dev_orient, joint1_pos, joint2_pos, win_size, win_slide, z1_est, z2_est)

if nargin < 6, z1_est = []; end
if nargin < 7, z2_est = []; end

rot_idxs = [1 win_size]; % one window apart
rot_device = dev_orient(rot_idxs(2))*conj(dev_orient(rot_idxs(1))); % R*Q1 = Q2 --> R = Q2*Q1'


% assume the position is given as x, y
% invert y-axis?? determine which way z is pointing!
limb_span1 = joint2_pos(rot_idxs(1),:) -joint1_pos(rot_idxs(1),:);
limb_span2 = joint2_pos(rot_idxs(2),:) -joint1_pos(rot_idxs(2),:);

angle1 = atan2(limb_span1(2), limb_span1(1));
angle2 = atan2(limb_span2(2), limb_span2(1));

cam_rot_angle = angle2-angle1;

% the change in angle is due to a twist along the camera normal,(camera z-axis)
% the change in length is due to a swing around an axis lying in the camera (xy) plane
%[c 0 0 s]
cam_twist = quaternion([cos(cam_rot_angle/2) 0 0 sin(cam_rot_angle/2)]);
% R = ST --> S = RT', where R is rotation in camera frame
% S = [w_S, x_S, y_S, z_S];
%   = [w_R*w_T + zR*zT, xR*wT-yR*zT, xR*zT+yR*wT, -wR*zT - zR*wT]
%   = [wR*c + zR*s, xR*c-yR*s, xR*s+yR*c, -wR*s-zR*c]
% also w_S = cos(theta/2), where cos(theta) = change in length/full limb length
% let's search for the camera swing that matches the orientation change?
% i.e. wST == wOrient?

% given Q = TS
% we know two things, QL_1Q' = L_2 and r1^2+z1^2 = r2^+z2^2

% let's find a z1 and z2 and make a swing vector that satisfies what we see
% on screen
[x1, v, f1, f2] = make_swing(cam_twist, limb_span1, limb_span2, z1_est, z2_est);
z1 = x1(5);
z2 = x1(6);


cam_swing = quaternion(x1(1:4)./norm(x1(1:4)));

d2cEst = cam_twist*cam_swing*conj(rot_device);
end


function [S, fVal, exitflag, output] = make_swing(twist, l1, l2, z1_est, z2_est)
l_limb1 = hypot(l1(1),l1(2));
l_limb2 = hypot(l2(1), l2(2));

if nargin < 4 || isempty(z1_est), z1_est = 1; end
if nargin < 5 || isempty(z2_est), z2_est = 1; end

    function [c, ceq] = inner_opt_const(x)
       % z^2 + l_limb^2 = actual_limb_length^2
       z1 = x(5);
       z2 = x(6);
       ceq = (z1^2 + l_limb1^2)-(z2^2+l_limb2^2);
       c = [];
    end

    function [score] = inner_opt_fun(x)
        S_hat = x(1:4);
        z2 = x(6);
        z1 = x(5);
        
        limb_vector1 = [0 l1 z1];
        limb_vector2 = [0 l2 z2];
        
        axis1 = quaternion(limb_vector1./norm(limb_vector1)); %[ 0 x1 y1 z1]
        axis2 = quaternion(limb_vector2./norm(limb_vector2)); %[ 0 x2 y2 z2]
        % we are looking for z1 and z2 and also...
        % we assume we swing then twist 
        % we are trying to find S` = [a, -b, -c, -d] such that
        % S*limb1*S' = T'*limb2*T
        % so basically to maximize the dot product of LHS*RHS
        
        
        S_hat = quaternion(S_hat./norm(S_hat));
        q1 = conj(twist)*quaternion(axis2)*twist;
        q2 = (S_hat)*quaternion(axis1)*conj(S_hat);
        [w1, x1, y1, z1] = parts(q1);
        [w2, x2, y2, z2] = parts(q2);
        cs = dot([w1, x1, y1, z1], [w2, x2, y2, z2]);
        score = 1-abs(cs);
    end


[S, fVal, exitflag, output] = fmincon(@inner_opt_fun, [1 0 0 0 z1_est z2_est], [],[],[],[],[-1 -1 -1 -1 -100 -100], [1 1 1 1 100 100], @inner_opt_const);

end



function [ang, ax] = quat_angle_axis(q)
[w,x,y,z] = parts(q);
ang = acos(w)*2;
ax = [x y z]./sin(ang);
ax = ax./norm(ax);
end