function [dataIMU, dataCams] = parse_total_capture(basedir, activityName, subjectPrefix) 
% get the dataIMU and dataCams from Xsens and h5 files

activityImuFile = [basedir '\' subjectPrefix '\' activityName '_xsens_auxfields.sensors'];
activityCamPrefix = [basedir '\' activityName '\TC_' subjectPrefix '_' activityName];
camCalibFile = [basedir '\camCalibration.txt'];


%activityIMUfile, activityCamPrefix, camCalibFile

%addpath(genpath('.'));	% Make sure all folders and subfolders are added to the path
camCalib = readTotalCaptureCamCalib(camCalibFile);

dataIMU = readXsens(activityImuFile);
%dataCams = struct();	
    for iCam = 1:8
    % Process cam data if needed (otherwise it has already been loaded)
    activityCamFile = [activityCamPrefix '_cam' num2str(iCam) '.h5'];
        if exist(activityCamFile, 'file')~=2  % Make sure cam file exists (I haven't processed some camera angles [on purpose]) otherwise just ignore this
            fprintf('Couldn''t find file %s, skipping!\n', activityCamFile);
            continue;
        end
        aux = getPosFromCam(activityCamFile, [], true);
        aux.fps = 60;
        aux.params.cam = camCalib(iCam);
        dataCams(iCam) = aux;
    end
end



