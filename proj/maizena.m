%% projecto de PIV
% Diogo Gois
% João Ramiro
% José Miragaia
% 1ª parte, os valores da matriz de rotação e translação entre as câmaras
% são introduzidas manualmente, sendo que neste caso se vai fazer apenas o
% LOAD destas matrizes

%%
clear 
close all

% Directório onde se encontram as imagens de profundidade e de rgb para
% análise
myDir_prof = '../maizena/';

%guardar todos os vectores de profundidade da camera 1
prof_a=dir('../maizena/depth1*.mat');

%inicializar a matriz que vai coter momentaneamente os valores das imagens
%de profundidade 
imgmed = zeros(480,640,length(prof_a));

% agrupar todas as imagens de profundidade para que seja possivel encontrar
% o background das imagens de cada 1 das cameras:
for i=1:length(prof_a)
    load([myDir_prof prof_a(i).name])
    imgmed(:,:,i) = double(depth_array)/1000;% divide-se por 100 pois nós queremos os dados em metros e nos ficheiros vêem em milimetros
end

%background camera a
backGround_a = median(imgmed,3);% faz a mediana da imagem ao longo do tempo(3ª dimensão), logo irá criar uma imagem que corresponde aos valores mais comuns de cada pixel para a imagem de profundidade da câmera 1



%guardar todos os vectores de profundidade da camera 2
prof_b=dir('../maizena/depth2*.mat');
for i=1:length(prof_b)
    load( [myDir_prof prof_b(i).name])
    imgmed(:,:,i) = double(depth_array)/1000;% agrupar as imagens de profundidade ca camera 2
end

% background camera b
backGround_b = median(imgmed,3);% faz a mediana da imagem ao longo do tempo(3ª dimensão), logo irá detetar o background


% load the rgb images to analise 
% directório e inicio do nome em que se irão encontras as imagens a ser analisadas
myDir = '../maizena/rgb_image';
ext_img = '.png.';%extensão dos ficheiros da imagem rgb a ser analisada

%temos que alterar isto apra que as matrizes R e T corretas, podemos fazer
%a função procrustes do professor de forma a termos/ ou feita com o VL feat
% retornar em tr.T a matriz de rotação e tr.c a de translação
load ../maizena/rly_close.mat;
% introduzir aqui o codigo do procrustes -> ou refazer os alinhamento pela
% função procrustesfalso.m

%percorrer todas as imagens
for i =1:length(prof_b)
    
    %load da imagem de profundidade da camera 1, a divisão por 1000 vem do
    %facto de queremos em metros
    load([myDir_prof 'depth1_' int2str(i) '.mat'])
    deptharray1 = double(depth_array)/1000;
    
    % profundidade camera 2
    load([myDir_prof 'depth2_' int2str(i) '.mat'])
    deptharray2 = double(depth_array)/1000;
    
    %read rgb image 1
    im1 = imread([myDir '1_' int2str(i) ext_img]);
    %read rgb image 1
    im2 = imread([myDir '2_' int2str(i) ext_img]);
    

    % mostar as duas imagens de forma ao utilizador se aperceber qual o frame a ser analizado
    figure (1)
    imagesc(im1)%imagem 1
    figure (2)
    pause(1)
    imagesc(im2)%imagem 2
    pause(1)
    


    %os valores de retorno são:
    %ra: as linhas que pertencem ao objecto,
    %ca: as colunas que pertencem ao objecto
    %flag1 serve para verificar se foi encontrado algum objecto 
    [ra, ca, flag1] = getextremes_depth(deptharray1, backGround_a);%esta função serve para encontrar os objectos pertecencentes ao background e trazer os locais(pixeis(linha e coluna)) onde estes se encontram

    % analizar na outra imagem
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
 

%  %verificar pq às vezes pode acontecer coisas estranhas(0, locais muito longe)
%  x_max = max(Extremes(:,1));
%  x_min = min(Extremes(:,1));
%  y_max = max(Extremes(:,2));
%  y_min = min(Extremes(:,2));
%  z_max = max(Extremes(:,3));
%  z_min = min(Extremes(:,3));

    % se não foi encontrado nenhum objecto então não é necessária fazer
    % mais análises
    if (flag1 == 0 || flag2 == 0)
        continue;
    end
   
 %Compute XYZ from depth image (u,v) and depth z(u,v)- CHECK FILE
 
    %calcular a posição 3D de todos os pontos pertencentes à imagem 1,
    %começando por analisar no dominio da camera de profundidade
    xyz1=get_xyzasus(deptharray1(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);

    %Compute "virtual image" aligned with depth
    % juntar valores rgb para os valores 3D calculados anteriormente
    rgbd1=get_rgbd(xyz1,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);

    %imagem 2
    %repetir criação dos pontos 3d para a segunda camara
    imagesc(deptharray2)

    %Compute XYZ from depth image (u,v) and depth z(u,v)- CHECK FILE
    xyz2=get_xyzasus(deptharray2(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);

    %Compute "virtual image" aligned with depth
    rgbd2=get_rgbd(xyz2,im2,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
    
    

    % gerar a point cloud para a camera 1 a partir dos valores calculados
    % anteriormente, considerando que é na camera 1 que se encontra o
    % referencial a ser usado para a caracterização do espaço
    pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
    
    
    % gerar a pointcloud, a partir dos pontos gerados com o seu referencial
    % e depois é aplicado as matrizes de rotação e translação
    pc2=pointCloud(xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:),'Color',reshape(rgbd2,[480*640 3]));
    
    %guardar os valores em 3D pertencentes ao objecto detetado, da camera 1
    foreground = pc1.Location((480 * ra + ca),:);
    
    %aqui da camera 2
    foreground = [foreground ; pc2.Location((480 * rb + cb),:)];
    
    %aqui remove-se os pontos que se encontram mal medidos (alguns pontos da camera não recebem de volta a radiação emitida )
    foreground = foreground(foreground(:,3)>0.1,:);
    
    
    %ir procurar os valores maximos e minimos que pertencem ao objecto
    % sendo possivel criar uma caixa que englobe objecto
    Maxs = max(foreground);%(retira o maximo valor de cada coordenada)
    Mins = min(foreground);%(retira o mínimo valor de cada coordenada)

    
 end

