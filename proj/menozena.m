
%%
% clear 
close all

% Direct�rio onde se encontram as imagens de profundidade e de rgb para
% an�lise
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
%for i =1:length(prof_b)
i= 9;
    
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
    

    % mostar as duas imagens de forma ao utilizador se aperceber qual o frame a ser analizado
%      figure
%      imagesc(im1)%imagem 1
%      figure
%      imagesc(im2)%imagem 2
%      
    
    %background subtraction (retor uns ou zeros)
    fg1 = abs(double(deptharray1) - backGround_a)>0.25;
    fg2 = abs(double(deptharray2) - backGround_b)>0.25;
    
    %elimina os zeros do foreground (fg fica com fator de escala marado but its ok)
    fg1= fg1 .*double(deptharray1);
    fg2= fg2 .*double(deptharray2);
    
     figure
     imagesc(deptharray1);
     figure
     imagesc(deptharray2);
    
     
     %%
     %INSERIR AQUI AS MERDAS PARA O GRADIENTE FICAR FIXE
     
     
     
     %%
     
      figure
     imagesc(fg1);
      figure
     imagesc(fg2);
    %%
    %faz black and white label
    bw1 = bwlabel(fg1,8);
    bw2 = bwlabel(fg2,8);
     figure
     imagesc(bw1);
     figure
     imagesc(bw2);

 %encontra labels maiores que 1000
   labelCounts1=tabulate(bw1(:));
    labelCounts2=tabulate(bw2(:));
   
    %devolve o indice do label e consequentemente o label
    goodLabels1 =find(labelCounts1(:,2)>1000)-1;
    goodLabels2 =find(labelCounts2(:,2)>1000)-1;
    
    %removes bg from goodlabels encontra o max das counts e remove o dos
    %good labels
 goodLabels2(find(max(labelCounts2(:,2))))=[];
  goodLabels1(find(max(labelCounts1(:,2))))=[];
  
  
  %gera imagem onde 1 corresponde aos  pixeis os objetos else sao 0
  goodItems1=zeros(480,640);
  for i=1:length(goodLabels1)
  goodItems1=goodItems1 | bw1==goodLabels1(i);
  end
figure
imagesc(goodItems1);

  goodItems2=zeros(480,640);
  for i=1:length(goodLabels2)
  goodItems2=goodItems2 | bw2==goodLabels2(i);
  end
figure
imagesc(goodItems2);


%vai buscar ao depth array apenas os objetos bons
depthArrayFG1= deptharray1.*goodItems1;
depthArrayFG2= deptharray2.*goodItems2;

%  figure
% imagesc(depthArrayFG1);
% figure
% imagesc(depthArrayFG2);

%converte para 3D tais pixeis
xyzFG1=get_xyzasus(depthArrayFG1(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
xyzFG2=get_xyzasus(depthArrayFG2(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);


%elimina zeros dos pontos 3d
 xyzFG1(  all(~xyzFG1,2), :  ) = [];
 xyzFG2(  all(~xyzFG2,2), :  ) = [];

%faz point clouds dos foregrounds e faz merge destes
pcFG1=pointCloud(xyzFG1);
xyz2FGin1FG=xyzFG2*tr.T+ones(length(xyzFG2),1)*tr.c(1,:);
 
pcFG2=pointCloud(xyz2FGin1FG);
%       figure
%     pcshow(pc1);
%     figure
%     pcshow(pc2);
figure
pcshow(pcmerge(pcFG1,pcFG2,0.001));