myDir = '../';
ext_img = '.jpg.';%extensão dos ficheiros da imagem rgb a ser analisada


% introduzir aqui o codigo do procrustes -> ou refazer os alinhamento pela
% função procrustesfalso.m

close all
%percorrer todas as imagens


i=1;
    %load da imagem de profundidade da camera 1, a divisão por 1000 vem do
    %facto de queremos em metros
  
    %read rgb image 1
    im1 = imread([myDir 'copos' ext_img]);
    %read rgb image 1

    
    imagesc(im1 )
    
    
    hsvImage = rgb2hsv(im1);
    figure
    imagesc(hsvImage);
    
    hue=hsvImage(:,:,1);
    
    figure
    imagesc(hue);
    
     sat=hsvImage(:,:,2);
    
    figure
    imagesc(sat);
    
     val=hsvImage(:,:,3);
    
    figure
    imagesc(val);
    
    mean(hue(:))