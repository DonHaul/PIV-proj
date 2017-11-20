clear;
close all; 

im1 = imread('../imagens/data_rgb/rgb_image1_1.png');
im2 = imread('../imagens/data_rgb/rgb_image2_1.png');

load ../imagens/data_rgb/depth1_1.mat;
deptharray1 = depth_array;
load ../imagens/data_rgb/depth2_1.mat; 
deptharray2 = depth_array;
load ../imagens/matlab.mat;







%Compute XYZ from depth image (u,v) and depth z(u,v)- CHECK FILE
xyz1=get_xyzasus(deptharray1(:),[480 640],1:640*480,Depth_cam.K,1,0);

%Compute "virtual image" aligned with depth
rgbd1=get_rgbd(xyz1,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);

%imagem 2

imagesc(deptharray2)

%Compute XYZ from depth image (u,v) and depth z(u,v)- CHECK FILE
xyz2=get_xyzasus(deptharray2(:),[480 640],1:640*480,Depth_cam.K,1,0);

%Compute "virtual image" aligned with depth
rgbd2=get_rgbd(xyz2,im2,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);


pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
pc2=pointCloud(xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:),'Color',reshape(rgbd2,[480*640 3]));
figure 
hold off;
%showPointCloud(pc1)
pcshow(pcmerge(pc1,pc2,0.001));
