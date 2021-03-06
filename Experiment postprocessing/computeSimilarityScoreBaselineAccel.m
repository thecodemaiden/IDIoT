function score = computeSimilarityScoreBaselineAccel(magnitudesIoT, posCamJoints3D, alphaPosDeviceAlongBodyPart, tInds, movAvgWinLength, derivWinLength)
	if nargin<3 || isempty(alphaPosDeviceAlongBodyPart), alphaPosDeviceAlongBodyPart = 0.5; end
	if nargin<4 || isempty(tInds), tInds = true(1,length(posCamJoints3D)); end
	if nargin<5 || isempty(movAvgWinLength), movAvgWinLength = 15; end
	if nargin<6 || isempty(derivWinLength), derivWinLength = 45; end

	if isfield(magnitudesIoT, 'linearAccel')
		linearAccelIoT = magnitudesIoT.linearAccel(tInds,:);
	else
		linearAccelIoT = magnitudesIoT.accel(tInds,:) - rotateframe(magnitudesIoT.quat(tInds,:), [0 0 9.81]);
	end
	linearAccelFiltered = movingAvgFilter(movAvgWinLength, linearAccelIoT);
	devicePosInCam3D = posCamJoints3D(tInds,1:3) + alphaPosDeviceAlongBodyPart.*(posCamJoints3D(tInds,4:6)-posCamJoints3D(tInds,1:3));
	devicePosInCam3D(:,2) = -devicePosInCam3D(:,2);  % Flip the y
	deviceAccelInCam3D = derivFilter(devicePosInCam3D', 2, 1, 2, derivWinLength)';
	validInds = ~isnan(deviceAccelInCam3D(:,1));
	if sum(validInds) < length(validInds)/4
		score = 1;
		fprintf('Too few accel data points (%d out of %d). Score: 1\n', sum(validInds), length(validInds));
	else
		if false
			R = rigid_transform_3D(linearAccelFiltered(validInds,:), deviceAccelInCam3D(validInds,:));
			linearAccelIoTtoCam = R*linearAccelFiltered(validInds,:)';
			score = sum(diag(pdist2(linearAccelIoTtoCam, deviceAccelInCam3D(validInds,:)', 'correlation')))/6;
		else
			%score = pdist2(sqrt(sum(linearAccelFiltered(validInds,:).^2, 2))', sqrt(sum(deviceAccelInCam3D(validInds,:).^2, 2))', 'correlation');
			score = min(1-xcorr(sqrt(sum(linearAccelFiltered(validInds,:).^2, 2)), sqrt(sum(deviceAccelInCam3D(validInds,:).^2, 2)), 'coeff'));  % 1-maxCrossCorr so that lower is better
		end
	end
end
