

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

backGround_a = median(imgmed,3);% faz a mediana da imagem ao longo do tempo(3ª dimensão), logo irá detetar o back ground

prof_b=dir('../maizena/depth2*.mat');
for i=1:length(prof_b)
    load( [myDir_prof prof_b(i).name])
    imgmed(:,:,i) = double(depth_array)/1000;
end

backGround_b = median(imgmed,3);% faz a mediana da imagem ao longo do tempo(3ª dimensão), logo irá detetar o back ground



myDir = '../maizena/rgb_image';
ext_img = '.png.';

load ../imagens/matlab.mat;

for i =1:length(prof_b)
    
    
    load([myDir_prof 'depth1_' int2str(i) '.mat'])
    deptharray_a = double(depth_array)/1000;
    
    
    load([myDir_prof 'depth2_' int2str(i) '.mat'])
    deptharray_b = double(depth_array)/1000;
    
    camera_a = imread([myDir '1_' int2str(i) ext_img]);
    camera_b = imread([myDir '2_' int2str(i) ext_img]);
    figure (1)
    imagesc(camera_a)
    figure (2)
    pause(1)
    imagesc(camera_b)
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

    
end
% temos os nomes das imagens guardadas em camera_a/b e prof_a/b



