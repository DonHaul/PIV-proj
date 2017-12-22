%% projecto de PIV
% Diogo Gois
% João Ramiro
% José Miragaia
% 1ª parte, os valores da matriz de rotação e translação entre as câmaras
% são introduzidas manualmente, sendo que neste caso se vai fazer apenas o
% LOAD destas matrizes




% eixo que é criado a partir da camera 1 o y aumenta para baixo, o x
% aumenta para a direita e o z aumenta na profundidade.

%%
clear 
close all

%varivel que conta o numero total de objectos encontrados, conta em
%repetido para diferenetes imagens(tempos_diferentes
obj = 1;
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
%for i =1:length(prof_b)
i= 9;
    
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
    %pause(1)
    imagesc(im2)%imagem 2
   % pause(1)
    


 

    
    
    
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
%     if (flag_1 == 0 || flag_2 == 0)
%         continue;
%     end
   
 %Compute XYZ from depth image (u,v) and depth z(u,v)- CHECK FILE
 
    %calcular a posição 3D de todos os pontos pertencentes à imagem 1,
    %começando por analisar no dominio da camera de profundidade
    xyz1=get_xyzasus(deptharray1(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);

    %Compute "virtual image" aligned with depth
    % juntar valores rgb para os valores 3D calculados anteriormente
    rgbd1=get_rgbd(xyz1,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);

    %imagem 2
    %repetir criação dos pontos 3d para a segunda camara
    %imagesc(deptharray2)

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
    
    
    
       %os valores de retorno são:
    %ra: as linhas que pertencem ao objecto,
    %ca: as colunas que pertencem ao objecto
    %flag1 serve para verificar se foi encontrado algum objecto 
    
    fg1 = abs(double(deptharray1) - backGround_a)>0.25;
    fg2 = abs(double(deptharray2) - backGround_b)>0.25;
    figure
    imagesc(fg1);
    figure
    imagesc(fg2);
    
    bw1 = bwlabel(fg1,8);
    bw2 = bwlabel(fg2,8);
    figure
    imagesc(bw1);
    figure
    imagesc(bw2);
    
    %%
    [ra, ca, flag_1,size_1] = getextremes_depth(deptharray1, backGround_a);%esta função serve para encontrar os objectos pertecencentes ao background e trazer os locais(pixeis(linha e coluna)) onde estes se encontram

    % analizar na outra imagem
    [rb, cb, flag_2,size_2] = getextremes_depth(deptharray2, backGround_b);
    
    
    %%
    fim = 0;
    % percorre todos os objectos encontrado pela camera 1
    for object=1:flag_1
        %guardar os valores em 3D pertencentes ao objecto detetado, da camera 1
        %foreground_1 = pc1.Location((640 * ra(fim+1:fim+size_1(object)) + ca(fim+1:fim+size_1(object))),:);%vai seleccionar as linhas linhas que pertencem ao objecto , começando pela linha que vem aseguir ao ultimo elemtento do objecto anterior
        foreground_1 = pc1.Location(640 * ra + ca,:);%vai seleccionar as linhas linhas que pertencem ao objecto , começando pela linha que vem aseguir ao ultimo elemtento do objecto anterior

        %aqui da camera 2
       % foreground_2 = pc2.Location((640 * rb(fim+1:fim+size_2(object)) + cb(fim+1:fim+size_2(object))),:);%vai seleccionar as linhas linhas que pertencem ao objecto , começando pela linha que vem aseguir ao ultimo elemtento do objecto anterior
        foreground_2 = pc2.Location(640 * rb + cb,:);%vai seleccionar as linhas linhas que pertencem ao objecto , começando pela linha que vem aseguir ao ultimo elemtento do objecto anterior

        
        %aqui remove-se os pontos que se encontram mal medidos (alguns pontos da camera não recebem de volta a radiação emitida  z=0)
%         foreground_1 = foreground_1(foreground_1(:,3)>0.1,:);
%         foreground_2 = foreground_2(foreground_2(:,3)>0.1,:);
%         
        pointclound1 = pointCloud(foreground_1);
        pointclound2 = pointCloud(foreground_2);
        figure
        pcshow(pcmerge(pointclound1,pointclound2,0.001));
        
        %analisar aqui que se os dois foregrounds pertencem ao mesmo object
        %caso contrário pensar numa solução, porenquanto assuminos que o
        %tamanho é relativamente igual para os dois casos.

        %ir procurar os valores maximos e minimos que pertencem ao objecto
        % sendo possivel criar uma caixa que englobe objecto
        Maxs(obj,:) = max([foreground_1;foreground_2]);%(retira o maximo valor de cada coordenada)
        Mins(obj,:) = min([foreground_1;foreground_2]);%(retira o mínimo valor de cada coordenada)
        %criar independentes valores de maximos e frame a ser analizado
        obj = obj+1;
    end

    
% end

