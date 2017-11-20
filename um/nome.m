
% carrega todos os nomes das imagens/ rgb
d = dir ('*.jpg');

IMGS = zeros (480,640,length(d));% inicializa para não demorar muito tempo
IMGSS = IMGS;
for (i=1:length(d))
    IM = imread(d(i).name);
    IMGS(:,:,i) = double(rgb2gray(IM));
    imagesc(IMGS(:,:,i));
    % aqui temos todas as imagens em gray (em itensidade)
    colormap(gray);
    pause()
end


%%
d = dir ('*.mat');

IMG = zeros (480,640,length(d));% inicializa para não demorar muito tempo
for (i=1:length(d))
    load (d(i).name);
    IMGS(:,:,i) = double(depth_array);
    imagesc(IMGS(:,:,i));
    % aqui temos todas as imagens em gray (em itensidade)
    colormap(gray);
    pause()
end





backGround = median(IMGS,3); % acho que não é assim que se faz, o comparação deve ocorrer na profundidade
%%
for i = 1: length(d)

    IMG(:,:,i) = abs(IMGS(:,:,i) - backGround)>320; % diferenças de 25 cm não são registadas logos por exemplos os pés casualmente não serão representados
    % aqui temos a imagem
    %RES(i) = bwconncomp(IMG(:,:,i));
    % aqui os conjuntos 
    %IMGSS(:,:,i) = labelmatrix(RES(i));
    
    IMG(:,:,i) = bwlabel(IMG(:,:,i));
    % IMG(:,:,i) = im2bw(IMG(:,:,i),0.1);
    % IMG(:,:,i) = imbinarize(IMG(:,:,i));
    % this is spoopi
%    IMG(:,:,i) = squeeze(IMG(:,:,i));
   imagesc(IMG(:,:,i));
   % imshow(label2rgb(IMG(:,:,i)));
    
   % colormap(gray);
    pause()
    
    GMI = IMG(:,:,1);
    MGI(i,:) = GMI(:);
    [n,bin] = hist(MGI(i,:),unique(MGI(i,:)));
    [~,idx] = sort(-n);
    n(idx)
    bin(idx)
    

end

% tirar o maximo e ver o segundo maximo
hist(GMI(:), 0:255);

% %%
% imgmed=zeros(480,640,38);
% for i=1:38,
%     load(['depth1_' int2str(i) '.mat']);
%     imagesc(depth_array);
%     imgmed(:,:,i)=double(depth_array)/1000;
%     pause(.1)
%     drawnow;
% end
% bg=median(imgmed,3);
% d=dir('depth1*.mat');
% for i=1:length(d),
%     load(d(i).name);
%     imagesc(abs(double(depth_array)/1000-bg)>.25);
%     colormap(gray);
%     pause(1);
% end
%     

%%
% 
% HISTO = [];
% 
% for i =1:RES.NumObjects
% 
%     HISTO = [HISTO ; RES(1).  ];
% 
% end