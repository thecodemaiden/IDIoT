% try to estimate real world joint positions based on vicon gt
%1) we know pos of accel, so we need to extrapolate out from that point
%based on the device orientation

cam = 1;
limb_ori = compare_ori.L_UpArm.vicon(1);
limb_alpha = 0.4; % shoulder - elbow

limb_gt_pos = compare_pos.L_UpArm(cam).vicon_bone(1,:);
ax = limb_ori*quaternion([0 0 0 1])*conj(limb_ori); % TODO: is this the right axis?
[w,x,y,z] = parts(ax);
limb_ax = [x y z]./norm([x y z]);

world_to_cam = [dataCams(cam).params.cam.intrinsicMat [0 ; 0 ; 0]]*dataCams(cam).params.cam.extrinsicMat;

j1_pos = dataCams(cam).camPos(1).lShoulder.pos2D(1,:);
j2_pos = dataCams(cam).camPos(1).lElbow.pos2D(1,:);

b = world_to_cam*[limb_ax';0];
A = [j1_pos' j2_pos'; 1 1];

x = A\b;

world_accel_pos = vicon_pos.LeftArm(1,:);
