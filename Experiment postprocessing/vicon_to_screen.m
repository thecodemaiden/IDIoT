function [vicon_px] = vicon_to_screen(dataCams, vicon_pos)

n_parts = length(vicon_pos);
n_cams = length(dataCams);


vicon_px = struct();

for cc = 1:n_cams
    cam_mat = [dataCams(cc).params.cam.intrinsicMat [0;0;0]]*dataCams(cc).params.cam.extrinsicMat;
    for pp = 1:n_parts
        if cc == 1
            vicon_px(pp).body_part = vicon_pos(pp).body_part;
            vicon_px(pp).cams = cell(n_cams, 1);
        end
        n_frames = length(vicon_pos(pp).pos);
        vpos = [vicon_pos(pp).pos'; ones(1, n_frames)];
        hom = cam_mat * vpos;
        px_loc = (hom./hom(3,:))';
        vicon_px(pp).cams{cc} = px_loc(:,1:2);
    end
end
        
end