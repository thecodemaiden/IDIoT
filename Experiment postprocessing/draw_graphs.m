% %plot some new figures such as longest contiguous window length
% figure;
% longest_win = cellfun(@max, chunk_idxs);
% for ii=1:4
%     subplot(2,2,ii);
%     bar(longest_win(:,2*ii-1:2*ii)./60);
%     xticks(1:25);
%     xticklabels(body_parts);
%     ylabel('Longest contiguous detection (s)');
%     set(gca, 'XTickLabelRotation', 60);
%     ylim([0 70]);
%     hold on
%     plot([0 26], [4115 4115]/60, 'k--')
%     xlim([0.5 25.5]);
%     legend(sprintf('Camera %d', 2*ii-1), sprintf('Camera %d', 2*ii));
% 
% end
% 
% % when the two body parts are both ddetected
% bp_temp = cell2struct([{'Head', 'nose','neck', -1}; {'Head2','lEar','rEar',0.5}; {'Sternum', 'rShoulder','lShoulder', 0.5}; {'Pelvis', 'rHip','lHip', 0.5}; {'L_UpArm', 'lShoulder','lElbow', 0.4}; {'R_UpArm', 'rShoulder','rElbow', 0.4}; {'L_LowArm', 'lElbow','lWrist', 0.8}; {'R_LowArm', 'rElbow','rWrist', 0.8}; {'L_UpLeg', 'lHip','lKnee', 0.5}; {'R_UpLeg', 'rHip','rKnee', 0.5}; {'L_LowLeg', 'lKnee','lAnkle', 0.5}; {'R_LowLeg', 'rKnee','rAnkle', 0.5}; {'L_Foot', 'lAnkle','lSmallToe', 0.5}; {'L_Foot2', 'lHeel', 'lBigToe', 0.5}; {'R_Foot', 'rAnkle','rSmallToe', 0.5}; {'R_Foot2', 'rHeel', 'rBigToe', 0.5}], {'bodyPart', 'joint1', 'joint2', 'alpha'}, 2);
% n_parts = length(bp_temp);
% display_names = {bp_temp.bodyPart};
% n_cams = 8;
% limb_windows = zeros(n_parts,n_cams);
% window_starts = zeros(n_parts,n_cams);
% for cc = 1:n_cams
%     for ii=1:n_parts
%             j1_missing = isnan(dataCams(cc).camPos(1).(bp_temp(ii).joint1).pos2D(:,1));
%             j2_missing = isnan(dataCams(cc).camPos(1).(bp_temp(ii).joint2).pos2D(:,1));
% 
%             limb_missing = j1_missing | j2_missing;
% 
%             detect_change = diff(limb_missing);
%             start_is_missing = (limb_missing(1));
%             end_idx = length(limb_missing);
%             nan_to_real = find(detect_change == -1);
%             real_to_nan = find(detect_change == 1);
%             % Do I need to check if it was nan at the start?
%             minlen = min(length(nan_to_real),length(real_to_nan));
% 
%             if minlen == 0
%                 % there were no changes in the whole file
%                 if start_is_missing
%                     ranges = 0;
%                 else
%                     ranges = end_idx;
%                 end
%             else
%                 if ~start_is_missing
%                     real_to_nan = [real_to_nan; end_idx];
%                     nan_to_real = [0; nan_to_real];
%                 end
%                 ranges = real_to_nan(1:minlen) - nan_to_real(1:minlen);
% 
%             end
%             [mv, mi] = max(ranges);
%             limb_windows(ii,cc) = mv;
%             if minlen > 0
%                 window_starts(ii,cc) = nan_to_real(mi)+1;
%             end
%     end
% end
% 
% figure;
% pretty_names = strrep(display_names, '_', '.');
% for cc = 1:n_cams
%     subplot(3,3,cc);
%     bar(limb_windows(:,cc)./60);
%     xticks(1:16);
%     xticklabels(pretty_names);
%     ylabel('Longest contiguous detection (s)');
%     set(gca, 'XTickLabelRotation', 60);
%     ylim([0 70]);
%     hold on
%     plot([0 17], [4115 4115]/60, 'k--')
%     xlim([0.5 16.5]);
%     title(sprintf('Camera %d', cc));
% 
% end
% 
% % try all possible head combos + a rotation matrix that relates the
% % original nose/neck to the new axis
% % similar for feet
% selected_parts = fieldnames(compare_pos);
% posErrByCam = cell(8,1);
% figure;
% for ii = 1:8
%     perr = zeros(4115,13);
%     subplot(3,3,ii)
%     for pp=1:13
%         bp = selected_parts{pp};
%         perr(:,pp) = compare_pos.(bp)(ii).pos_err;
%     end
%     boxplot(perr);
%     xticks(1:13);


error_correction =

%     xticklabels(selected_parts);
%     set(gca, 'XTickLabelRotation', 60);
%     title(sprintf('OpenPose limb position error for Camera %d',ii))
%     ylabel('Error (pixels)')
%     posErrByCam{ii}=perr;
% end
% selected_parts = fieldnames(compare_ori);
% oriError = zeros(4115,13);
% for pp=1:13
%     bp = selected_parts{pp};
%     oriError(:,pp) = compare_ori.(bp).rot_err;
% end
% 
% 


