% Copyright 2022 Paul Scholefield UK Centre for Ecology and hydrology

clear all; close all;

tic
display(sprintf('Running'))
fprintf(' \n')


%% Step 1: Read Image
% Bring an image into the workspace.

folders=([
'2021-10-06_11-52-31'])

for fol = 1:size(folders,1)

source=(['N:\Model Inputs\Yail Survey\Sentera\',folders(fol,:)])


cd([source,'\NDVI'])
a = dir('*.jpg');

n = numel(a);

for num = 1 : n
  

cd([source,'\NDVI'])

    valstr=num2str(num,'%05.f');
    filename=(['IMG_', valstr, '.jpg']);

original1 = imread(filename);
original=rgb2gray(original1);
[EXIFinfo1]=imfinfo(filename);
exposure1 = EXIFinfo1.DigitalCamera.ExposureTime;
gain1=EXIFinfo1.DigitalCamera.ISOSpeedRatings;


%% Step 2: Resize and Rotate the Image

cd([source,'\RGB'])
distorted1 = imread(filename);
[EXIFinfo2]=imfinfo(filename);
exposure2 = EXIFinfo2.DigitalCamera.ExposureTime;
gain2=EXIFinfo2.DigitalCamera.ISOSpeedRatings;

distorted=rgb2gray(distorted1);


%% Step 3: Find Matching Features Between Images
% Detect features in both images.
ptsOriginal  = detectSURFFeatures(original);
ptsDistorted = detectSURFFeatures(distorted);

%%
% Extract feature descriptors.
[featuresOriginal,  validPtsOriginal]  = extractFeatures(original,  ptsOriginal);
[featuresDistorted, validPtsDistorted] = extractFeatures(distorted, ptsDistorted);

%%
% Match features by using their descriptors.
indexPairs = matchFeatures(featuresOriginal, featuresDistorted);

%%
% Retrieve locations of corresponding points for each image.
matchedOriginal  = validPtsOriginal(indexPairs(:,1));
matchedDistorted = validPtsDistorted(indexPairs(:,2));

%% Step 4: Estimate Transformation
% Find a transformation corresponding to the matching point pairs using the
% statistically robust M-estimator SAmple Consensus (MSAC) algorithm, which
% is a variant of the RANSAC algorithm. It removes outliers while computing
% the transformation matrix. You may see varying results of the
% transformation computation because of the random sampling employed by the
% MSAC algorithm.
[tform, inlierIdx] = estimateGeometricTransform2D(...
    matchedDistorted, matchedOriginal, 'similarity');
inlierDistorted = matchedDistorted(inlierIdx, :);
inlierOriginal  = matchedOriginal(inlierIdx, :);

%% Step 5: Solve for Scale and Angle
% Use the geometric transform, tform, to recover the scale and angle.
% Since we computed the transformation from the distorted to the original
% image, we need to compute its inverse to recover the distortion.
%
%  Let sc = s*cos(theta)
%  Let ss = s*sin(theta)
%
%  Then, Tinv = [sc -ss  0;
%                ss  sc  0;
%                tx  ty  1]
%
%  where tx and ty are x and y translations, respectively.
%

%%
% Compute the inverse transformation matrix.
Tinv  = tform.invert.T;

ss = Tinv(2,1);
sc = Tinv(1,1);
scaleRecovered = sqrt(ss*ss + sc*sc);
thetaRecovered = atan2(ss,sc)*180/pi;

%%
% The recovered values should match your scale and angle values selected in
% *Step 2: Resize and Rotate the Image*.

%% Step 6: Recover the Original Image
% Recover the original image by transforming the distorted image.
outputView = imref2d(size(original));
recovered1  = imwarp(distorted1,tform,'OutputView',outputView);

R_dn = recovered1(:, :, 1);
G_dn = recovered1(:, :, 2);
B_dn = recovered1(:, :, 3);
RE_dn = original1(:, :, 1);
%G2=original1(:, :, 2);
NIR_dn = original1(:, :, 3);

%%%% from Sentera
%%RGB Camera 1
B_eo = B_dn / (gain1 * exposure1);
G_eo = G_dn / (gain1 * exposure1);
R_eo = R_dn / (gain1 * exposure1);

%%NDRE Camera 2
RE_eo = RE_dn / (gain2 * exposure2);
NIR_eo = NIR_dn / (gain2 * exposure2);

%%% RGB Camera 1
B = 1.377*B_eo - 0.182 * G_eo - 0.061 * R_eo;
G = -0.199*B_eo + 1.420*G_eo - 0.329 * R_eo;
R = -0.034*B_eo - 0.110* G_eo + 1.150 * R_eo;

%%NDRE Camera 2
RE   = 2.7 * (-0.956 * NIR_eo + 1.000 * RE_eo);
NIR = 2.7 * (2.426 * NIR_eo - 0.341 * RE_eo);

multispec = cat(3,R,G,B,RE,NIR);
filename6=(['IMG_', valstr, '_M_new.tif']);

options.overwrite = 'true';
options.message =false;

cd([source,'\STACKED'])
%image=multispec;
saveastiff(multispec,filename6,options);

formatSpec = 'Processing %1.0f out of %1.0f folder %1.0f \n';
fprintf(formatSpec,num,n,fol)

end
end

toc


