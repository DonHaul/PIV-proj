function [ FG_pts , obj_num ] = getForeGroundpts( backGround_a,backGround_b,deptharray1,deptharray2,im1,im2 )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

load ../maizena/rly_close.mat;

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
     %INSERIR AQUI AS MERDAS... PARA O GRADIENTE FICAR FIXE done prob 
%      
     [Gmag,Gdir] = imgradient(fg1);% magnitude e direcção do gradiente de cada uma das imagens
     Gmag(Gmag<0.3)=0;%remove os valores muito baixos do gradiente para que alterações insignificantes sejam ignorados
     fg1 = fg1.*~Gmag;% locais de elevado gradiente vão passar a ter 0, causando que os considremos como background, logo as edges de cada objecto localizado na imagem sejam identificados.
     [Gmag,Gdir] = imgradient(fg2);
     Gmag(Gmag<0.3)=0;
     fg2 = fg2.*~Gmag;
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

FG_pts=pcmerge(pcFG1,pcFG2,0.001);
pcshow(FG_pts);

obj_num = max(length(goodLabels1),length(goodLabels2));

end

