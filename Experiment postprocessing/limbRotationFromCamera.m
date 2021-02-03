function [in_plane, out_plane] = limbRotationFromCamera(posCamJoints)
% posCamJoints is Nx4: u_joint1 v_joint1 u_joint2 v_joint1
% the change in limb slope is due to rotation in the camera plane, i.e.
% around the normal axis vector. The change in apparent length is due to
% rotation out of the plane, i.e. around a vector 'parallel' to the plane
% in_plane corresponds to a twist around normal to camera plane
% out_plane is then the swing around a vector lying in the camera plane

% get the slopes of all limbs, as angles
du = posCamJoints(:,3)-posCamJoints(:,1);
dv = posCamJoints(:,4)-posCamJoints(:,2);

angles = atan2(dv,du);
lengths = hypot(dv,du);

in_plane =  diff(angles,[],1);
out_plane = diff(lengths,[],1);

% convert to an angle?
% assume the change in length is related to the cos of the swing angle
% check this assumption in math!!
out_plane = acos(out_plane ./ lengths(1:end-1));
end
