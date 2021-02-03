function [new_cost] = exclude_from_cost_matrix(already_assigned)
% Assumes each row is a body part IMU and each column is an openpose-based limb
% to assign
new_cost = already_assigned;

end