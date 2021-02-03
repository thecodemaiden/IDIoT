% try to find the stablest conversion from rotation in cam frame --> dev
% frame
% assuming the 'default' axis of a limb is [ 0 0 1 0] in world frame, then
% in the camera frame a real-world rotation of R_w would give a camera
% frame orientation of A_c = R_w2c*R_w*Quat([0 0 1 0])*conj(R_w)*conj(R_w2c)
% And a device frame orientation of A_d = R_w2d*R_w*Quat([0 0 1 0])*conj(R_w)*conj(R_w2d)
%
% =>    conj(R_w2c)A_c*R_w2c = R_w*Quat([0 0 1 0])*conj(R_w)
% and   conj(R_w2d)A_d*R_w2d = R_w*Quat([0 0 1 0])*conj(R_w) = A_w
% so A_c = R_w2c*conj(R_w2d)*A_d*R_w2d*conj(R_w2c)
% and A_c = [0 unit(joint2 - joint1)]
% camera frame: K*[xc; yc; zc] = zc*[u; v; 1]
% =>    [xc1; yc1; zc1] = zc1*inv(K)*[u1; v1; 1]
% =>     A_c = [0; unit(zc2*inv(K)*[u2;v2;1] - zc1*inv(K)*[u1; v1; 1])]