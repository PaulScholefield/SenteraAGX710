% Copyright 2022 Paul Scholefield UK Centre for Ecology and hydrology

clear; close all; clc;

tic
disp('Running');
fprintf(' \n');

% Define folders
folders = {'2021-10-06_11-52-31'};

for fol = 1:numel(folders)
    source = fullfile('N:\Model Inputs\Yail Survey\Sentera\', folders{fol});
    ndviFolder = fullfile(source, 'NDVI');
    rgbFolder = fullfile(source, 'RGB');
    stackedFolder = fullfile(source, 'STACKED');

    % Read and process images
    cd(ndviFolder);
    imgFiles = dir('*.jpg');
    numFiles = numel(imgFiles);

    for num = 1:numFiles
        filename = sprintf('IMG_%05d.jpg', num);

        % Read original NDVI image
        original1 = imread(fullfile(ndviFolder, filename));
        original = rgb2gray(original1);
        EXIFinfo1 = imfinfo(filename);
        exposure1 = EXIFinfo1.DigitalCamera.ExposureTime;
        gain1 = EXIFinfo1.DigitalCamera.ISOSpeedRatings;

        % Read distorted RGB image
        distorted1 = imread(fullfile(rgbFolder, filename));
        EXIFinfo2 = imfinfo(filename);
        exposure2 = EXIFinfo2.DigitalCamera.ExposureTime;
        gain2 = EXIFinfo2.DigitalCamera.ISOSpeedRatings;
        distorted = rgb2gray(distorted1);

        % Detect and match features
        ptsOriginal = detectSURFFeatures(original);
        ptsDistorted = detectSURFFeatures(distorted);
        [featuresOriginal, validPtsOriginal] = extractFeatures(original, ptsOriginal);
        [featuresDistorted, validPtsDistorted] = extractFeatures(distorted, ptsDistorted);
        indexPairs = matchFeatures(featuresOriginal, featuresDistorted);
        matchedOriginal = validPtsOriginal(indexPairs(:,1));
        matchedDistorted = validPtsDistorted(indexPairs(:,2));

        % Estimate transformation
        [tform, inlierIdx] = estimateGeometricTransform2D(matchedDistorted, matchedOriginal, 'similarity');
        inlierDistorted = matchedDistorted(inlierIdx, :);
        inlierOriginal = matchedOriginal(inlierIdx, :);

        % Solve for scale and angle
        Tinv = tform.invert.T;
        ss = Tinv(2,1);
        sc = Tinv(1,1);
        scaleRecovered = sqrt(ss*ss + sc*sc);
        thetaRecovered = atan2(ss, sc) * 180 / pi;

        % Recover the original image
        outputView = imref2d(size(original));
        recovered1 = imwarp(distorted1, tform, 'OutputView', outputView);

        % Calibrate and stack images
        R_dn = recovered1(:, :, 1);
        G_dn = recovered1(:, :, 2);
        B_dn = recovered1(:, :, 3);
        RE_dn = original1(:, :, 1);
        NIR_dn = original1(:, :, 3);

        B_eo = B_dn / (gain1 * exposure1);
        G_eo = G_dn / (gain1 * exposure1);
        R_eo = R_dn / (gain1 * exposure1);
        RE_eo = RE_dn / (gain2 * exposure2);
        NIR_eo = NIR_dn / (gain2 * exposure2);

        B = 1.377 * B_eo - 0.182 * G_eo - 0.061 * R_eo;
        G = -0.199 * B_eo + 1.420 * G_eo - 0.329 * R_eo;
        R = -0.034 * B_eo - 0.110 * G_eo + 1.150 * R_eo;
        RE = 2.7 * (-0.956 * NIR_eo + 1.000 * RE_eo);
        NIR = 2.7 * (2.426 * NIR_eo - 0.341 * RE_eo);

        multispec = cat(3, R, G, B, RE, NIR);
        filename6 = fullfile(stackedFolder, sprintf('IMG_%05d_M_new.tif', num));

        options.overwrite = true;
        options.message = false;
        saveastiff(multispec, filename6, options);

        fprintf('Processing %d out of %d in folder %d\n', num, numFiles, fol);
    end
end

toc



