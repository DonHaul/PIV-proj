
%%
clear
close all

% Directório onde se encontram as imagens de profundidade e de rgb para
% análise
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
num_obj_prev = {};
  frames_obj1_prev =[];

for i =1:length(prof_b)
   % i=6;
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
    
    if(i ==18)
        1+1
    end
    
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
        
        
     
        
        deptharray_obj = deptharray1.*frames_obj1(:,:,k);
        xyz1=get_xyzasus(deptharray_obj(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        
        rgbd=get_rgbd(xyz1,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
        
        figure
        imagesc(rgbd)
        
       
        hsvImage = rgb2hsv(rgbd);
        hImage = hsvImage(:,:,1);
        hue_image=hImage(:);
         hue_image(  all(~hue_image,2), :  ) = [];
         mean(hue_image);
         ramiro(k).hue_score = mean(hue_image);
     
        
          moldura((minimum(1)-3:maximum(1)+30),(minimum(2)-3):(maximum(2)+3))=1;
        
        deptharray_obj = deptharray1.*moldura;
        xyz1=get_xyzasus(deptharray_obj(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        
        rgbd=get_rgbd(xyz1,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
        
         I = single(rgb2gray(rgbd));
        %I1 = single(rgb2gray(im1));
        
        %[fa,da] = vl_sift(I1);
        [ramiro(k).f,ramiro(k).d] = vl_sift(I);
        
        
   end
    par = zeros(size(frames_obj2,3),size(frames_obj1,3));%verse dimensao ta ccerto grafo
    p = 1;
    encontrado = [];
   for k = 1 :size(frames_obj2,3)
       [row,col]=find(frames_obj2(:,:,k)==1);
        
     	maximum = max([row';col']');
        minimum = min([row';col']');
        moldura = zeros(480,640);
        
        moldura((minimum(1)-3:maximum(1)+30),(minimum(2)-3):(maximum(2)+3))=1;
        
        
        deptharray_obj = deptharray2.*moldura;
        xyz1=get_xyzasus(deptharray_obj(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        
        rgbd=get_rgbd(xyz1,im2,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
     
        
        I = single(rgb2gray(rgbd));
        %I1 = single(rgb2gray(im1));
        
        %[fa,da] = vl_sift(I1);
        [gois(k).f,gois(k).d] = vl_sift(I);
        
        moldura((minimum(1)-3:maximum(1)+30),(minimum(2)-3):(maximum(2)+3))=1;
        
        
        deptharray_obj = deptharray2.*frames_obj2(:,:,k);
        xyz1=get_xyzasus(deptharray_obj(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        
     
        
        rgbd=get_rgbd(xyz1,im2,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
        
              
     figure
         imagesc(rgbd)
        
        hsvImage = rgb2hsv(rgbd);
        hImage = hsvImage(:,:,1);
        hue_image=hImage(:);
         hue_image(  all(~hue_image,2), :  ) = [];
         mean(hue_image);
         gois(k).hue_score = mean(hue_image);
     
   
         
        
        high_score=0;
        calc_score = 0;
        %compara os do obj1 com os do 2 (loopl dentro do loop)
       for j = 1:size(frames_obj1,3)
           if(find(encontrado == j))
               continue;
           end
          [matches,scores] = vl_ubcmatch(gois(k).d,ramiro(j).d);
          
          calc_score = length(matches);
          calc_score = calc_score + 10*log(abs((1/(gois(k).hue_score - ramiro(j).hue_score))));
          
          
          
          if (high_score < calc_score)
                high_score = calc_score;
                %i
                found = j;
                encontrado(p) = found;
              
          end
       end
           
            if(high_score < 8 )
                %obj está sozinho hehe
                objects(length(objects)+1).descriptor = d;%aqui vai ser inserido o primeiro objecto log nas linhas seguintes nõa vai ser preciso por o factor +1 nas linhas seguintes
                objects(length(objects)).X = [];
                objects(length(objects)).Y = [];
                objects(length(objects)).Z = [];
            
                objects(length(objects)).frames_tracked = i;
            else
                %está nas duas imagens
                %EMPARELHATE
                par(k, found) = 1;
                  p = p+1;
               
            end
            
   
       
       
   
   end

   B = any(par);
   p = 0;
        close all  
   for n=1:size(frames_obj2,3)
       
    for m=1:size(frames_obj1,3)
        
        if(par(n,m)==1)
              p = p +1 ;
            miragaia(p).d = [ramiro(m).d , gois(n).d];  
            miragaia(p).hue_score = (ramiro(m).hue_score + gois(n).hue_score)/2;
          
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
    
    % está a mais acho eu
    if(par(n,m)==1)
        continue;
    end
    %emparelha
             disp('PIL')
           deptharray_obj2 = deptharray2.*frames_obj2(:,:,n);
            p = p +1 ;
            %objecto novo não é preciso concatenar
             miragaia(p).d =  gois(n).d; 
             miragaia(p).hue_score = gois(n).hue_score;
           
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
                
    
    
   end
   
    %vê se matriz par tem colunas a zero
    if B(n) == 0
        p = p +1 ;
        disp('NOT');
        
            %não é preciso concatenar
              miragaia(p).d = ramiro(m).d ;  
              miragaia(p).hue_score = ramiro(m).hue_score;
            
        deptharray_obj1 = deptharray1.*frames_obj1(:,:,m);
  
           
        xyz1=get_xyzasus(deptharray_obj1(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);

        
        %elimina zeros dos pontos 3d
        xyz1(  all(~xyz1,2), :  ) = [];

        %faz point clouds dos foregrounds e faz merge destes
        pcFG1=pointCloud(xyz1);

        
        FG_pts=pcFG1;
%         figure
%         pcshow(FG_pts);
%         
        exrtremes1 = max(FG_pts.Location);
        minions1 = min(FG_pts.Location);
%         
       miragaia(p).X = [minions1(1),minions1(1),minions1(1),minions1(1), exrtremes1(1) ,exrtremes1(1) ,exrtremes1(1) ,exrtremes1(1)];
        
       miragaia(p).Y = [minions1(2),minions1(2), exrtremes1(2) ,exrtremes1(2) ,minions1(2),minions1(2),exrtremes1(2) ,exrtremes1(2)];
        
       miragaia(p).Z = [minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3)];        
    end
     
        

    
    
%codigo relevante tracking
if ~isempty(  frames_obj1_prev )
    encontrado = [];
    p =1;
    for m=1:length(miragaia)
        
        num_matches = 0;
        
        for n=1:length(miragaia_prev)
          %  calc_score = 0;
          
             if(find(encontrado == n))
                 continue;
             end
          
            [matches,scores] = vl_ubcmatch(miragaia_prev(n).d,miragaia(m).d);
           % no caso dos miragaias acho que vai ser mais importante se nõa
           % separarmos entre as imagesn
             calc_score = length(matches);
          
            calc_score = calc_score + 10*log(abs((1/(miragaia(m).hue_score - miragaia_prev(n).hue_score))));
            
            if num_matches < calc_score
                num_matches = calc_score;
             found = n; %found e o antigo
             encontrado(p) = found;
             
             end  
        end
            if num_matches > 30
              %adicionar ao obj m
                objects(miragaia_prev(found).obj_prev).X=[objects(miragaia_prev(found).obj_prev).X ; miragaia(m).X];
                objects(miragaia_prev(found).obj_prev).Y=[objects(miragaia_prev(found).obj_prev).Y ; miragaia(m).Y];
                objects(miragaia_prev(found).obj_prev).Z=[objects(miragaia_prev(found).obj_prev).Z ; miragaia(m).Z];
                objects(miragaia_prev(found).obj_prev).frames_tracked = [objects(miragaia_prev(found).obj_prev).frames_tracked,i];
                miragaia(m).obj_prev = miragaia_prev(found).obj_prev;
                p = p+1;
                
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
%load da imagem de profundidade da camera 1, a divisão por 1000 vem do
    %facto de queremos em metros
    
   
    
        
        for i=1:11
         %   figure
            hold on
             for m = 1:length(objects)
                 frame = find(objects(m).frames_tracked==i);
                 if(frame)
                      plot3(objects(m).X(frame,:) , objects(m).Y(frame,:),objects(m).Z(frame,:),'*')
                 end
             end
                     
    
    load([myDir_prof 'depth1_' int2str(i) '.mat'])
    deptharray1 = double(depth_array)/1000;
    
    % profundidade camera 2
    load([myDir_prof 'depth2_' int2str(i) '.mat'])
    deptharray2 = double(depth_array)/1000;
    
    %read rgb image 1
    im1 = imread([myDir '1_' int2str(i) ext_img]);
    %read rgb image 1
    
    im2 = imread([myDir '2_' int2str(i) ext_img]);
% 
%     figure(1);
% imagesc([im1 im2]);
% figure(2);
% imagesc([deptharray1 deptharray2]);
xyz1=get_xyzasus(deptharray1(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
xyz2=get_xyzasus(deptharray2(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);


cl1=reshape(im1,480*640,3);
cl2=reshape(im2,480*640,3);

p1=pointCloud(xyz1,'Color',cl1);
p2=pointCloud(xyz2,'Color',cl2);
% 
% figure(3)
% showPointCloud(p1);
% figure(4)
% showPointCloud(p2);

xyz2in1=xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:);
 

p2=pointCloud(xyz2in1,'Color',cl2);


mergedpc=pcmerge(p1,p2,0.001);
%         figure
        pcshow(mergedpc);
%         hold on
      
        
    
%         
%      figure(1);
%      
%      figure(2).
%     im1 =imread( imgseq1(objects(23).frames_tracked(k)).rgb)
%     imshow(im1)
% pause();
end
