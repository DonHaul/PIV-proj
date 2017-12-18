%%
% READ IMAGES and GENERATE POINT CLOUDS
load ../imagens/matlab.mat;
im1=imread('../maizena/rgb_image1_3.png');
im2=imread('../maizena/rgb_image2_3.png');
load('../maizena/depth1_3.mat')
dep1=depth_array;
load('../maizena/depth2_3.mat')
dep2=depth_array;
%dep2(find(dep2(:)>4000))=0;
xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
%REGISTER RGB TO DEPTH
rgbd1 = get_rgbd(xyz1, im1, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
rgbd2 = get_rgbd(xyz2, im2, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
figure(1);imagesc(rgbd1 );
figure(2);imagesc(rgbd2 );



