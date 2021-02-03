function [twist, swing] = twist_swing_decomposition(q, v)
% http://allenchou.net/2018/05/game-math-swing-twist-interpolation-sterp/
% q can be recovered as sw*tw

q = (1./norm(q))*q;
v= (1./norm(v))*v;

tol = eps; % TODO: is this big enough?
[wq, xq, yq, zq] = parts(q);
old_axis = [xq, yq, zq];

if norm(old_axis).^2 < tol
    % we have a potential singularity
    [~, xr, yr, zr] = parts(q * quaternion([0 v]));
    swing_axis = cross(v, [xr yr zr]);
    
    if norm(swing_axis).^2 < tol
        % another potential singularity
        swing = quaternion([1 0 0 0]);
    else
        swing_angle = acos(dot(v, [xr yr zr])/(norm(v)*norm([xr yr zr])));
        s = sin(swing_angle); c = cos(swing_angle);
        swing_axis = swing_axis ./ norm(swing_axis);
        swing = quaternion([c s*swing_axis]);
    end
    twist = quaternion([-1 v]);
else
    % get the projection of old_axis on v
    proj_len = dot(v, old_axis);
    projected = proj_len .* v;

    twist_components = [wq projected];
    if proj_len < 0
        twist = quaternion(-twist_components);
    else
        twist = quaternion(twist_components);
    end
    twist = 1./norm(twist)*twist;

    swing = q * conj(twist);
end
end
