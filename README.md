# SenteraAGX710
Image stacking and transformation in Matlab, plus exposure correction for the Sentera AGX710 5 band camera.

The Sentera AGX710 5 band multispectral camera takes two images. 1 is RGB, the second is RE/G/NIR. 
The two images are taken a few millseconds apart, and each has EXIF information recorded in the JPEG Metadata.
In order to make use of the 5 bands in photogrammetry napplications such as Agisoft Metashape, the JPEG images must be stacked into 5 band TIFF format images. 
This Matlab 2022 code uses an affine transformation to adjust the position of the second image, and locate it. The EXIF information is also used to correct the exposure
of the RE/NIR data so the 5 layers are compatible and suitable for NDVI calculations.

Simply locate the folder for your data and add this to the path information in the code. 
Normally, RGB and NDVI are located in the parent folder, then add a folder called STACKED.

Run the code. You should find a set of stacked TIFFs once processing is complete.
