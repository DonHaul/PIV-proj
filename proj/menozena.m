
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
    imgmed(:,:,i) = double(depth_array)/1000;% divide-se por 1000 pois nós queremos os dados em metros e nos ficheiros vêem em milimetros
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


%% aqui para cima fazer função de descobriri background

%
% figure
%     imagesc(backGround_a)
% figure
%    imagesc(backGround_b)

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

descriptors=[];
objects_size = 0;
objects = {};
for i =9:length(prof_b)
    
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
    
    [FG_pts,num_obj,depthArrayFG1,depthArrayFG2,frames_obj] = getForeGroundpts(backGround_a,backGround_b,deptharray1,deptharray2,im1,im2);
  
        
    
    for k = 1:num_obj
%         figure
%         imagesc(frames_obj(:,:,k));
        [row,col]=find(frames_obj(:,:,k)==1);
        maximum = max([row';col']');
        minimum = min([row';col']');
        moldura = zeros(480,640);
        
        moldura((minimum(1)-3:maximum(1)+30),(minimum(2)-3):(maximum(2)+3))=1;
        
        
        
        
        deptharray_obj = deptharray1.*moldura;
        xyz1=get_xyzasus(deptharray_obj(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        
        rgbd=get_rgbd(xyz1,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
        
       % imagesc(rgbd)
        
        I = single(rgb2gray(rgbd));
        %I1 = single(rgb2gray(im1));
        
        %[fa,da] = vl_sift(I1);
        [f,d] = vl_sift(I);
        
        high_score =0;
        
        for j = 1:length(objects)
            [matches,scores] = vl_ubcmatch(d,objects(j).descriptor);
   
            if (high_score < length(matches))
                high_score = length(matches);
                %i
                found = j;
            end
        end
        
        if(high_score < 10)
            objects(length(objects)+1).descriptor = d;%aqui vai ser inserido o primeiro objecto log nas linhas seguintes nõa vai ser preciso por o factor +1 nas linhas seguintes
            objects(length(objects)).X = [];
            objects(length(objects)).Y = [];
            objects(length(objects)).Z = [];
            
            objects(length(objects)).frames_tracked = i;
        else
            objects(found).frames_tracked =[objects(found).frames_tracked , i] ;
        %         perm = randperm(size(fb,2));
        %         h1 = vl_plotframe(fb(:,perm));
        end
    end
    
    
    
    %[matches,scores] = vl_ubcmatch(da,db);
    
    
    
    pontos=FG_pts.Location;
    
    IDX = kmeans(pontos,num_obj);
    
    for m=1: num_obj
        obj=pontos(find(IDX==m),:);
        
        exrtremes1 = max(obj);
        minions1 = min(obj);
%         
        objects(m).X = [objects(m).X;minions1(1),minions1(1),minions1(1),minions1(1), exrtremes1(1) ,exrtremes1(1) ,exrtremes1(1) ,exrtremes1(1)];
        
        objects(m).Y = [objects(m).Y;minions1(2),minions1(2), exrtremes1(2) ,exrtremes1(2) ,minions1(2),minions1(2),exrtremes1(2) ,exrtremes1(2)];
        
        objects(m).Z = [objects(m).Z;minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3)];
        
%         
        
    end
    
    
    
end


