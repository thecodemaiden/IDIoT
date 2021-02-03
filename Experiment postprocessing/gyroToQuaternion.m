function [qr] = gyroToQuaternion(gyro, dt)
% Integrates the gyro vector to estimate a change in angle
% The gyro data is Nx3; columns are angular velocity components
if nargin < 2, dt = 1; end %use sampling interval
omega = sqrt(gyro(:,1).^2 + gyro(:,2).^2 + gyro(:,3).^2); % size of overall rotation
ax_norm = gyro ./ omega;
qr = quaternion([ cos(omega*dt/2) ax_norm(:,1).*sin(omega*dt/2) ax_norm(:,2).*sin(omega*dt/2) ax_norm(:,3).*sin(omega*dt/2) ]);

end