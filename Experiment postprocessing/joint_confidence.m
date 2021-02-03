nFrames = 4115;
conf_scores = zeros(nFrames,25);
h5_file = 's1_acting\TC_S1_acting1_cam1.h5';
for ii=1:nFrames
    fname = sprintf('/joint_list/frame%05d', ii);
    joint_info = h5read(h5_file, fname);
    conf_scores(ii, :) = joint_info(3,:);
end