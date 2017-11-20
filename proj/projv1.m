im1 = imread('../imagens/rgb_image_10.png');
im2 = imread('../imagens/rgb_image_14.png');

load ../imagens/depth_10.mat;
deptharray1 = depth_array;
load ../imagens/depth_14.mat; 
deptharray2 = depth_array;
load ../imagens/CalibrationData.mat;



imagesc(deptharray1)

%Compute XYZ from depth image (u,v) and depth z(u,v)- CHECK FILE
xyz1=get_xyzasus(deptharray1(:),[480 640],1:640*480,Depth_cam.K,1,0);
figure
%Display point cloud
p=pointCloud(xyz1);
showPointCloud(p)
%Read RGB image

figure;
imagesc(im1);
%Compute "virtual image" aligned with depth
rgbd=get_rgbd(xyz1,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
figure
imagesc([im1; rgbd])
cl=reshape(rgbd,480*640,3);
p1=pointCloud(xyz1,'Color',cl);
figure
% Point cloud with colour per pixel
showPointCloud(p1)



%imagem 2

imagesc(deptharray2)

%Compute XYZ from depth image (u,v) and depth z(u,v)- CHECK FILE
xyz2=get_xyzasus(deptharray2(:),[480 640],1:640*480,Depth_cam.K,1,0);
figure
%Display point cloud
p=pointCloud(xyz2);
showPointCloud(p)
%Read RGB image

figure;
imagesc(im2);
%Compute "virtual image" aligned with depth
rgbd=get_rgbd(xyz2,im2,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
figure
imagesc([im2; rgbd])
cl=reshape(rgbd,480*640,3);

p2=pointCloud(xyz2,'Color',cl);
figure
% Point cloud with colour per pixel
showPointCloud(p2)


pc2=pointCloud(xyz2,'Color',reshape(im2,[480*640 3]));
pc1=pointCloud(xyz1*Rcalib+ones(length(xyz1),1)*Tcalib','Color',reshape(im1,[480*640 3]));
figure(1);hold off;
%showPointCloud(pc1)
pcshow(pcmerge(pc1,pc2,0.001));
