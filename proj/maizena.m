

    %% aqui fui so eu(miragaia)
%restart with same dataset meaning using the same r and t matrixes

clear 
close all

myDir_prof = '../maizena/';

prof_a=dir('../maizena/depth1*.mat');
imgmed = zeros(480,640,length(prof_a));


for i=1:length(prof_a)
    load([myDir_prof prof_a(i).name])
    imgmed(:,:,i) = double(depth_array)/1000;
end

backGround_a = median(imgmed,3);% faz a mediana da imagem ao longo do tempo(3� dimens�o), logo ir� detetar o back ground

prof_b=dir('../maizena/depth2*.mat');
for i=1:length(prof_b)
    load( [myDir_prof prof_b(i).name])
    imgmed(:,:,i) = double(depth_array)/1000;
end

backGround_b = median(imgmed,3);% faz a mediana da imagem ao longo do tempo(3� dimens�o), logo ir� detetar o back ground

myDir = '../maizena/rgb_image';
ext_img = '.png.';

%temos que alterar isto apra que as matrizes R e T corretas, podemos fazer
%a fun��o procrustes do professor de forma a termos/ ou feita com o VL feat
% retornar em tr.T a matriz de rota��o e tr.c a de transla��o
load ../maizena/maizena.mat;
% introduzir aqui o codigo do procrustes -> ou refazer os alinhamento pela
% fun��o procrustesfalso.m

for i =1:length(prof_b)
    
        load([myDir_prof 'depth1_' int2str(i) '.mat'])
    deptharray1 = double(depth_array)/1000;
    
    
    load([myDir_prof 'depth2_' int2str(i) '.mat'])
    deptharray2 = double(depth_array)/1000;
    
    im1 = imread([myDir '1_' int2str(i) ext_img]);
    im2 = imread([myDir '2_' int2str(i) ext_img]);
    figure (1)
    imagesc(im1)
    figure (2)
    pause(1)
    imagesc(im2)
    pause(1)
    
    [ra, ca, flag1] = getextremes_depth(deptharray1, backGround_a);
%     Files_a = zeros(1,4);
%     for quatro=1:4
%         Files_a(quatro) = [(Extremes(quatro,1)) + (fix(Extremes(quatro,2)/480)*480) + rem(Extremes(quatro,2),480)] ;
           
%     end
    
    [rb, cb, flag2] = getextremes_depth(deptharray2, backGround_b);
%     Files_b = zeros(1,4);
%     for quatro=1:4
%         Files_b(quatro) = [(Extremes(quatro,1)) + (fix(Extremes(quatro,2)/480)*480) + rem(Extremes(quatro,2),480)] ;
%     end
%     
%  
% if isequal(Extremes_a,Extremes_b,zeros(4,2))
%     continue;
% end
%     
% Extremes = zeros(8,3);
% for quatro=1:4
% 
%     if isequal(Extremes_a,zeros(4,2))
%             Extremes(quatro+4,:) = get_xyz(Extremes_b(quatro,:),deptharray_b(Extremes_b(quatro,2),Extremes_b(quatro,1)),Depth_cam.K)*tr.T + tr.c(1,:);
%     elseif isequal(Extremes_b,zeros(4,2))
%             Extremes(quatro,:) = get_xyz(Extremes_a(quatro,:),deptharray_a(Extremes_a(quatro,2),Extremes_a(quatro,1)),Depth_cam.K);
%     else
%     Extremes(quatro,:) = get_xyz(Extremes_a(quatro,:),deptharray_a(Extremes_a(quatro,2),Extremes_a(quatro,1)),Depth_cam.K);
%     Extremes(quatro+4,:) = get_xyz(Extremes_b(quatro,:),deptharray_b(Extremes_b(quatro,2),Extremes_b(quatro,1)),Depth_cam.K)*tr.T + tr.c(1,:);
%     end
% end
 

%  %verificar pq �s vezes pode acontecer coisas estranhas(0, locais muito longe)
%  x_max = max(Extremes(:,1));
%  x_min = min(Extremes(:,1));
%  y_max = max(Extremes(:,2));
%  y_min = min(Extremes(:,2));
%  z_max = max(Extremes(:,3));
%  z_min = min(Extremes(:,3));

    if (flag1 == 0 || flag2 == 0)
        continue;
        
    end
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
    foreground = pc1.Location((640 * ra + ca),:);
    foreground = [foreground ; pc2.Location((640 * rb + cb),:)];
    
    Maxs = max(foreground);
    Mins = min(foreground);
 
 
 
%% analisar aqui os resultados para ver se � objecto ou n�o
    %%so ate aqui importa acho eu

    
 end
% temos os nomes das imagens guardadas em camera_a/b e prof_a/b



