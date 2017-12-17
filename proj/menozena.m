
%%
clear
close all

% Direct�rio onde se encontram as imagens de profundidade e de rgb para
% an�lise
myDir_prof = '../maizena/';

%guardar todos os vectores de profundidade da camera 1
prof_a=dir('../maizena/depth1*.mat');

%inicializar a matriz que vai conter momentaneamente os valores das imagens
%de profundidade
imgmed = zeros(480,640,length(prof_a));

% agrupar todas as imagens de profundidade para que seja possivel encontrar
% o background das imagens de cada 1 das cameras:
for i=1:length(prof_a)
    load([myDir_prof prof_a(i).name])
    imgmed(:,:,i) = double(depth_array)/1000;% divide-se por 1000 pois n�s queremos os dados em metros e nos ficheiros v�em em milimetros
end

%background camera a
backGround_a = median(imgmed,3);% faz a mediana da imagem ao longo do tempo(3� dimens�o), logo ir� criar uma imagem que corresponde aos valores mais comuns de cada pixel para a imagem de profundidade da c�mera 1

%guardar todos os vectores de profundidade da camera 2
prof_b=dir('../maizena/depth2*.mat');
for i=1:length(prof_b)
    load( [myDir_prof prof_b(i).name])
    imgmed(:,:,i) = double(depth_array)/1000;% agrupar as imagens de profundidade ca camera 2
end

% background camera b
backGround_b = median(imgmed,3);% faz a mediana da imagem ao longo do tempo(3� dimens�o), logo ir� detetar o background


%% aqui para cima fazer fun��o de descobriri background

%
% figure
%     imagesc(backGround_a)
% figure
%    imagesc(backGround_b)

% load the rgb images to analise
% direct�rio e inicio do nome em que se ir�o encontras as imagens a ser analisadas
myDir = '../maizena/rgb_image';
ext_img = '.png.';%extens�o dos ficheiros da imagem rgb a ser analisada

%temos que alterar isto apra que as matrizes R e T corretas, podemos fazer
%a fun��o procrustes do professor de forma a termos/ ou feita com o VL feat
% retornar em tr.T a matriz de rota��o e tr.c a de transla��o
load ../maizena/rly_close.mat;
% introduzir aqui o codigo do procrustes -> ou refazer os alinhamento pela
% fun��o procrustesfalso.m

%percorrer todas as imagens

descriptors=[];
objects_size = 0;
objects = {};
num_obj_prev = {};
  frames_obj1_prev =[];

for i =1:length(prof_b)
    
    %load da imagem de profundidade da camera 1, a divis�o por 1000 vem do
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
    
    [FG_pts,depthArrayFG1,depthArrayFG2,frames_obj1, frames_obj2] = getForeGroundpts(backGround_a,backGround_b,deptharray1,deptharray2,im1,im2);
  
     if(isempty (frames_obj1))
       continue;
     end
%     Z = linkage(FG_pts.Location);
%     T = cluster(Z, 'cutoff', 0.05, 'criterion', 'distance') %5cm e nao 50
%     dendrogram(Z)
%     num_obj = max(T); 
    


  % d_1 = zeros(1,size(frames_obj1,3));
   %f_1 = zeros(1,size(frames_obj1,3));

   ramiro = {};
   gois = {};
   miragaia = {};
   for k = 1:size(frames_obj1,3)
        
        [row,col]=find(frames_obj1(:,:,k)==1);
        
     	maximum = max([row';col']');
        minimum = min([row';col']');
        moldura = zeros(480,640);
        
        moldura((minimum(1)-3:maximum(1)+30),(minimum(2)-3):(maximum(2)+3))=1;
        
        deptharray_obj = deptharray1.*moldura;
        xyz1=get_xyzasus(deptharray_obj(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        
        rgbd=get_rgbd(xyz1,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
        
%         figure
%         imagesc(rgbd)
        
        I = single(rgb2gray(rgbd));
        %I1 = single(rgb2gray(im1));
        
        %[fa,da] = vl_sift(I1);
        [ramiro(k).f,ramiro(k).d] = vl_sift(I);
        
   end
    par = zeros(size(frames_obj2,3),size(frames_obj1,3));%verse dimensao ta ccerto grafo
    
   for k = 1 :size(frames_obj2,3)
       [row,col]=find(frames_obj2(:,:,k)==1);
        
     	maximum = max([row';col']');
        minimum = min([row';col']');
        moldura = zeros(480,640);
        
        moldura((minimum(1)-3:maximum(1)+30),(minimum(2)-3):(maximum(2)+3))=1;
        
        deptharray_obj = deptharray2.*moldura;
        xyz1=get_xyzasus(deptharray_obj(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        
        rgbd=get_rgbd(xyz1,im2,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
        
%        figure
%         imagesc(rgbd)
        
        I = single(rgb2gray(rgbd));
        %I1 = single(rgb2gray(im1));
        
        %[fa,da] = vl_sift(I1);
        [gois(k).f,gois(k).d] = vl_sift(I);
   
        high_score=0;
        
        %compara os do obj1 com os do 2 (loopl dentro do loop)
       for j = 1:size(frames_obj1,3)
           
          [matches,scores] = vl_ubcmatch(gois(k).d,ramiro(j).d);
          
          if (high_score < length(matches))
                high_score = length(matches);
                %i
                found = j;
          end
       end
           
            if(high_score < 8 )
                %obj est� sozinho hehe
                objects(length(objects)+1).descriptor = d;%aqui vai ser inserido o primeiro objecto log nas linhas seguintes n�a vai ser preciso por o factor +1 nas linhas seguintes
                objects(length(objects)).X = [];
                objects(length(objects)).Y = [];
                objects(length(objects)).Z = [];
            
                objects(length(objects)).frames_tracked = i;
            else
                %est� nas duas imagens
                %EMPARELHATE
                par(k, found) = 1;
               
            end
            
       
       
       
   
   end
   
   B = any(par);
   p = 0;
   for n=1:size(frames_obj2,3)
       
    for m=1:size(frames_obj1,3)
        
        if(par(n,m)==1)
              p = p +1 ;
            miragaia(p).d = [ramiro(m).d , gois(n).d];  
          
        %emparelha
         deptharray_obj1 = deptharray1.*frames_obj1(:,:,m);
           deptharray_obj2 = deptharray2.*frames_obj2(:,:,n);
           
        xyz1=get_xyzasus(deptharray_obj1(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        xyz2=get_xyzasus(deptharray_obj2(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        
        %elimina zeros dos pontos 3d
        xyz1(  all(~xyz1,2), :  ) = [];
        xyz2(  all(~xyz2,2), :  ) = [];

        %faz point clouds dos foregrounds e faz merge destes
        pcFG1=pointCloud(xyz1);
        xyz2in1=xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:);
 
        pcFG2=pointCloud(xyz2in1);

        
        FG_pts=pcmerge(pcFG1,pcFG2,0.001);
%         figure
%         pcshow(FG_pts);
        
        exrtremes1 = max(FG_pts.Location);
        minions1 = min(FG_pts.Location);
%         
       miragaia(p).X = [minions1(1),minions1(1),minions1(1),minions1(1), exrtremes1(1) ,exrtremes1(1) ,exrtremes1(1) ,exrtremes1(1)];
        
       miragaia(p).Y = [minions1(2),minions1(2), exrtremes1(2) ,exrtremes1(2) ,minions1(2),minions1(2),exrtremes1(2) ,exrtremes1(2)];
        
       miragaia(p).Z = [minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3)];
        
        
        break;   
        
        end                
   
    end
    
    if(par(n,m)==1)
        continue;
    end
    %emparelha
             disp('PIL')
           deptharray_obj2 = deptharray2.*frames_obj2(:,:,n);
            p = p +1 ;
             miragaia(p).d =  gois(n).d;  
           
        xyz2=get_xyzasus(deptharray_obj2(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        
        %elimina zeros dos pontos 3d
    
        xyz2(  all(~xyz2,2), :  ) = [];

        %faz point clouds dos foregrounds e faz merge destes
        xyz2in1=xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:);
 
        pcFG2=pointCloud(xyz2in1);

        
        FG_pts = pcFG2;
%         figure
%         pcshow(FG_pts);
        
        exrtremes1 = max(FG_pts.Location);
        minions1 = min(FG_pts.Location);
%         
        miragaia(p).X = [minions1(1),minions1(1),minions1(1),minions1(1), exrtremes1(1) ,exrtremes1(1) ,exrtremes1(1) ,exrtremes1(1)];
        
       miragaia(p).Y = [minions1(2),minions1(2), exrtremes1(2) ,exrtremes1(2) ,minions1(2),minions1(2),exrtremes1(2) ,exrtremes1(2)];
        
       miragaia(p).Z = [minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3)];
        
    
    
   end
   
    
    if B(n) == 0
        p = p +1 ;
        disp('NOT');
              miragaia(p).d = ramiro(m).d ;  
            
        deptharray_obj1 = deptharray1.*frames_obj1(:,:,m);
  
           
        xyz1=get_xyzasus(deptharray_obj1(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);

        
        %elimina zeros dos pontos 3d
        xyz1(  all(~xyz1,2), :  ) = [];

        %faz point clouds dos foregrounds e faz merge destes
        pcFG1=pointCloud(xyz1);

        
        FG_pts=pcFG1;
        figure
        pcshow(FG_pts);
        
        exrtremes1 = max(FG_pts.Location);
        minions1 = min(FG_pts.Location);
%         
       miragaia(p).X = [minions1(1),minions1(1),minions1(1),minions1(1), exrtremes1(1) ,exrtremes1(1) ,exrtremes1(1) ,exrtremes1(1)];
        
       miragaia(p).Y = [minions1(2),minions1(2), exrtremes1(2) ,exrtremes1(2) ,minions1(2),minions1(2),exrtremes1(2) ,exrtremes1(2)];
        
       miragaia(p).Z = [minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3)];        
    end
     
        

    
    
%codigo relevante tracking
if ~isempty(  frames_obj1_prev )
    
    for m=1:length(miragaia)
        
        num_matches = 0;
        for n=1:length(miragaia_prev)
            
            [matches,scores] = vl_ubcmatch(miragaia_prev(n).d,miragaia(m).d);
            
            if num_matches < length(matches)
                num_matches = length(matches);
             found = n; %found e o antigo
             end  
        end
            if num_matches > 30
              %adicionar ao obj m
              
 
                objects(miragaia_prev(found).obj_prev).X=[objects(miragaia_prev(found).obj_prev).X ; miragaia(m).X];
                objects(miragaia_prev(found).obj_prev).Y=[objects(miragaia_prev(found).obj_prev).Y ; miragaia(m).Y];
                objects(miragaia_prev(found).obj_prev).Z=[objects(miragaia_prev(found).obj_prev).Z ; miragaia(m).Z];
                objects(miragaia_prev(found).obj_prev).frames_tracked = [objects(miragaia_prev(found).obj_prev).frames_tracked,i];
                miragaia(m).obj_prev = miragaia_prev(found).obj_prev;
            else 
                objects(length(objects)+1).frames_tracked = i;
                %ja adicionamos ja fica crto
                objects(length(objects)).X = miragaia(m).X;
                objects(length(objects)).Y = miragaia(m).Y;
                objects(length(objects)).Z = miragaia(m).Z;
                miragaia(m).obj_prev = length(objects);
            end
    end
    
else
  %primeira vez
  
  objects=miragaia;
  
  for t = 1:size(miragaia,2)
  miragaia(t).obj_prev = t;
  objects(t).frames_tracked = i;
  end
  
end
    



    


    

    miragaia_prev = miragaia;
    par_prev = par;
    FG_pts_prev = FG_pts;

    depthArrayFG1_prev = depthArrayFG1;
    depthArrayFG2_prev = depthArrayFG2;
    frames_obj1_prev = frames_obj1;
    frames_obj2_prev = frames_obj2;
    
end

    %%
%load da imagem de profundidade da camera 1, a divis�o por 1000 vem do
    %facto de queremos em metros
    i=9;
    
    load([myDir_prof 'depth1_' int2str(i) '.mat'])
    deptharray1 = double(depth_array)/1000;
    
    % profundidade camera 2
    load([myDir_prof 'depth2_' int2str(i) '.mat'])
    deptharray2 = double(depth_array)/1000;
    
    %read rgb image 1
    im1 = imread([myDir '1_' int2str(i) ext_img]);
    %read rgb image 1
    
    im2 = imread([myDir '2_' int2str(i) ext_img]);

    figure(1);
imagesc([im1 im2]);
figure(2);
imagesc([deptharray1 deptharray2]);
xyz1=get_xyzasus(deptharray1(:),[480 640],1:640*480,Depth_cam.K,1,0);
xyz2=get_xyzasus(deptharray2(:),[480 640],1:640*480,Depth_cam.K,1,0);


cl1=reshape(im1,480*640,3);
cl2=reshape(im2,480*640,3);

p1=pointCloud(xyz1,'Color',cl1);
p2=pointCloud(xyz2,'Color',cl2);

figure(3)
showPointCloud(p1);
figure(4)
showPointCloud(p2);

xyz2in1=xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:);
 

p2=pointCloud(xyz2in1,'Color',cl2);


pila=pcmerge(p1,p2,0.001);
        figure
        pcshow(pila);
