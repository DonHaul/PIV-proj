% clear;
% close all; 
% 
% im1 = imread('../imagens/data_rgb/rgb_image1_1.png');
% im2 = imread('../imagens/data_rgb/rgb_image2_1.png');
% 
% load ../imagens/data_rgb/depth1_1.mat;
% deptharray1 = depth_array;
% load ../imagens/data_rgb/depth2_1.mat; 
% deptharray2 = depth_array;
% load ../imagens/matlab.mat;
% 
% 
% 
% 
% 
% 
% 
% %Compute XYZ from depth image (u,v) and depth z(u,v)- CHECK FILE
% xyz1=get_xyzasus(deptharray1(:),[480 640],1:640*480,Depth_cam.K,1,0);
% 
% %Compute "virtual image" aligned with depth
% rgbd1=get_rgbd(xyz1,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
% 
% %imagem 2
% 
% imagesc(deptharray2)
% 
% %Compute XYZ from depth image (u,v) and depth z(u,v)- CHECK FILE
% xyz2=get_xyzasus(deptharray2(:),[480 640],1:640*480,Depth_cam.K,1,0);
% 
% %Compute "virtual image" aligned with depth
% rgbd2=get_rgbd(xyz2,im2,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
% 
% 
% pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
% pc2=pointCloud(xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:),'Color',reshape(rgbd2,[480*640 3]));
% figure 
% hold off;
% %showPointCloud(pc1)
% pcshow(pcmerge(pc1,pc2,0.001));

%% aqui fui so eu(miragaia)
%restart with same dataset meaning using the same r and t matrixes

clear 
close all

myDir_prof = '../imagens/data_rgb/';

prof_a=dir('../imagens/data_rgb/depth1*.mat');
imgmed = zeros(480,640,length(prof_a));
for i=1:length(prof_a)
    load([myDir_prof prof_a(i).name])
    imgmed(:,:,i) = double(depth_array)/1000;
end

backGround_a = median(imgmed,3);% faz a mediana da imagem ao longo do tempo(3ª dimensão), logo irá detetar o back ground

prof_b=dir('../imagens/data_rgb/depth2*.mat');
for i=1:length(prof_b)
    load( [myDir_prof prof_b(i).name])
    imgmed(:,:,i) = double(depth_array)/1000;
end

backGround_b = median(imgmed,3);% faz a mediana da imagem ao longo do tempo(3ª dimensão), logo irá detetar o back ground



myDir = '../imagens/data_rgb/rgb_image';
ext_img = '.png.';

load ../imagens/matlab.mat;

for i =1:length(prof_b)
    
    
    load([myDir_prof prof_a(i).name])
    deptharray_a = double(depth_array)/1000;
    
    
    load([myDir_prof prof_b(i).name])
    deptharray_b = double(depth_array)/1000;
    
    camera_a = imread([myDir '1_' int2str(i) ext_img]);
    camera_b = imread([myDir '2_' int2str(i) ext_img]);
    imagesc(camera_a)
    pause(1)
    
    Extremes_a = getextremes_depth(deptharray_a, backGround_a);
%     Files_a = zeros(1,4);
%     for quatro=1:4
%         Files_a(quatro) = [(Extremes(quatro,1)) + (fix(Extremes(quatro,2)/480)*480) + rem(Extremes(quatro,2),480)] ;
%     end
    
    Extremes_b = getextremes_depth(deptharray_b, backGround_b);
%     Files_b = zeros(1,4);
%     for quatro=1:4
%         Files_b(quatro) = [(Extremes(quatro,1)) + (fix(Extremes(quatro,2)/480)*480) + rem(Extremes(quatro,2),480)] ;
%     end
%     
%  
if isequal(Extremes_a,Extremes_b,zeros(4,2))
    continue;
end
    
Extremes = zeros(8,3);
for quatro=1:4

    if isequal(Extremes_a,zeros(4,2))
            Extremes(quatro+4,:) = get_xyz(Extremes_b(quatro,:),deptharray_b(Extremes_b(quatro,2),Extremes_b(quatro,1)),Depth_cam.K)*tr.T + tr.c(1,:);
    elseif isequal(Extremes_b,zeros(4,2))
            Extremes(quatro,:) = get_xyz(Extremes_a(quatro,:),deptharray_a(Extremes_a(quatro,2),Extremes_a(quatro,1)),Depth_cam.K);
    else
    Extremes(quatro,:) = get_xyz(Extremes_a(quatro,:),deptharray_a(Extremes_a(quatro,2),Extremes_a(quatro,1)),Depth_cam.K);
    Extremes(quatro+4,:) = get_xyz(Extremes_b(quatro,:),deptharray_b(Extremes_b(quatro,2),Extremes_b(quatro,1)),Depth_cam.K)*tr.T + tr.c(1,:);
    end
end
 
 Extremes

 
%% analisar aqui os resultados para ver se ó objecto ou não
    %%so ate aqui importa acho eu

    %%
    %Compute XYZ from depth image (u,v) and depth z(u,v)- CHECK FILE
%     xyz_a=get_xyzasus(deptharray_a(:),[480 640],1:640*480,Depth_cam.K,1,0);
% 
%     %Compute "virtual image" aligned with depth
%     rgbd_a=get_rgbd(xyz_a,camera_a,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
% 
%     %imagem 2
% 
%     %Compute XYZ from depth image (u,v) and depth z(u,v)- CHECK FILE
%     xyz_b=get_xyzasus(deptharray_b(:),[480 640],1:640*480,Depth_cam.K,1,0);
% 
%     %Compute "virtual image" aligned with depth
%     rgbd_b=get_rgbd(xyz_b,camera_b,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);


     
    
  %  pc1=pointCloud(xyz_a,'Color',reshape(rgbd_a,[480*640 3]));
   % pc2=pointCloud(xyz_b*tr.T+ones(length(xyz_b),1)*tr.c(1,:),'Color',reshape(rgbd_b,[480*640 3]));
   % figure 
    %hold off;
    %showPointCloud(pc1)
    % isto acho que já não é preciso pois não precisamos de mostar
    % serve so para ver que as matrizes estão bem
    %pcshow(pcmerge(pc1,pc2,0.001));
   %pause
  % drawnow;
  
  
  
    
end
% temos os nomes das imagens guardadas em camera_a/b e prof_a/b



