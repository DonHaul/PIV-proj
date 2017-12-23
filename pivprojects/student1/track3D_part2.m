function [objects,cam1toW, cam2toW] = track3D_part( imgseq1, imgseq2,  cam_params)
%track3D_part1 object traking with only rgb and depth sequences and intrinsic camera parameters
%   [objects,cam1toW, cam2toW] = track3D_part( imgseq1, imgseq2,  cam_params)
%
% outputs
%   objects -> estrutura que cont�m informa��o sobre os objetos encontrados :frames_tracked,X,Y,Z,
%       objects.frames_tracked cont�m os frames em que o objecto foi encontrado
%       objects.i para cada frame um vector 1x8 que cont�m a posi��o i de um dos extremos do objecto
%               i = X,Y,Z
%
%   camitoW -> estrutura que cont�m a transforma��o a ser aplicada aos
%   pontos origin�rio da c�mara i para estarem representados no eixo de coordenadas do mundo real
%
%
% inputs
%   imgseqi -> estrura que cont�m a localiza��o dos ficheiros a serem analisados obtidos a partir da camara i
%               i = 1,2
%       imgseqi.rgb localiza��o da imagem rgb da c�mara i
%       imgseqi.depth localiza��o da imagem de profundidade da c�mara i
%
%   cam_params estrutura com os parametrso intrinsicos e extrinsicos de um kinectd
%       cam_params.Kdepth matriz k dos parametros intrinsecos da c�mera de profundidade
%       cam_params.R matriz de rota��o da camara de cor para a de profundidade
%       cam_params.T vector de transla��o da camara de cor para a de profundidade
%       cam_params.Krgb matriz k dos parametros intrinsecos da c�mera de cor
%


%inicializar a matriz que vai conter momentaneamente os valores das imagens
%de profundidade
imgmed = zeros(480,640,length(imgseq1));
% agrupar todas as imagens de profundidade para que seja possivel encontrar
% o background das imagens de cada 1 das cameras:
for i=1:length(imgseq1)
    load(imgseq1(i).depth)
    imgmed(:,:,i) = double(depth_array)/1000;% divide-se por 1000 pois n�s queremos os dados em metros e nos ficheiros v�em em milimetros
end

%background camera a
backGround_a = median(imgmed,3);% faz a mediana da imagem ao longo do tempo(3� dimens�o), logo ir� criar uma imagem que corresponde aos valores mais comuns de cada pixel para a imagem de profundidade da c�mera 1

%guardar todos os vectores de profundidade da camera 2

for i=1:length(imgseq2)
    load( imgseq2(i).depth)
    imgmed(:,:,i) = double(depth_array)/1000;% agrupar as imagens de profundidade ca camera 2
end

% background camera b
backGround_b = median(imgmed,3);% faz a mediana da imagem ao longo do tempo(3� dimens�o), logo ir� detetar o background

%  j� n�o vai ser utilizada a vari�vel imgmed por isso pode ser apagada para que o programa corra mais suavemente
clear imgmed


% para ficar de acordo com os dados fornecidos
Depth_cam.K = cam_params.Kdepth;
R_d_to_rgb = cam_params.R;
T_d_to_rgb = cam_params.T;
RGB_cam.K  = cam_params.Krgb;

% fun��o que quando fornecido um par de imagens no mesmo instante de tempo e os parametros cas cameras retorna a matriz de rota��o de 1 para 2 a tranla��o de 1 para2
tr = procrustesverdadeiro(imgseq1(ceil(length(imgseq2)/2)),imgseq2(ceil(length(imgseq2)/2)),Depth_cam,R_d_to_rgb,T_d_to_rgb,RGB_cam);
%considera-se que em 1 � o centro dos eixos de oordenadas



%inicializa��es
descriptors=[];
objects_size = 0;
objects = {};
num_obj_prev = {};
frames_obj1_prev =[];
frames_obj2_prev =[];

%percorrer todas as imagens
for i =1:length(imgseq2)
    
    %load da imagem de profundidade da camera 1, a divis�o por 1000 vem do
    %facto de queremos em metros
   load( imgseq1(i).depth)
    deptharray1 = double(depth_array)/1000;
    
    % profundidade camera 2
    %load([myDir_prof 'depth2_' int2str(i) '.mat'])
    load( imgseq2(i).depth)
    deptharray2 = double(depth_array)/1000;
    
    %read rgb image 1 and 2
   im2 = imread(imgseq2(i).rgb);
   im1 = imread(imgseq1(i).rgb);
    
    % a partir das imagens de profundidade encontra-se uma imagem para cada objecto encontrado em camara
    [depthArrayFG1,depthArrayFG2,frames_obj1, frames_obj2] = getForeGroundpts(backGround_a,backGround_b,deptharray1,deptharray2);
  
    %inicializa�o das estruturas que cont�m informa��o sobre os onjectos
    %encontrados
    ramiro = {};
   gois = {};
   miragaia = {};
    
    
    % save computation if no objects were found
    if(isempty(frames_obj2) && isempty(frames_obj1))
           miragaia_prev = miragaia;
        continue;
    end

    % percorrer todos os conjuntos de pontos encontrados na c�mera 1
   for k = 1:size(frames_obj1,3)
         if(isempty(frames_obj1))
           break;
         end
       %encontrar os valores m�ximos dos pontos pertencentes aos objetos encontrados
        [row,col]=find(frames_obj1(:,:,k)==1);
        
     	maximum = max([row';col']');
        minimum = min([row';col']');
        moldura = zeros(480,640);
        
        
     
        
        deptharray_obj = deptharray1.*frames_obj1(:,:,k);
        xyz1=get_xyzasus(deptharray_obj(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        rgbd=get_rgbd(xyz1,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
        
       %retira a informa��o sobre a cor do objecto encontrado
        hsvImage = rgb2hsv(rgbd);
        hImage = hsvImage(:,:,1);
        hue_image=hImage(:);
         hue_image(  all(~hue_image,2), :  ) = [];
         mean(hue_image);
         % guarda a informa��o sobre a tonalidade do objecto
         ramiro(k).hue_score = mean(hue_image);
     
        
        
        
        moldx_min = (minimum(1)-3>1).*(minimum(1)-3)+ (~(minimum(1)-3>1)).*1;
        moldx_max = (maximum(1)+30<480).*(maximum(1)+30) +(~(maximum(1)+30<480)).*480;
        moldy_min = (minimum(2)-3>1).*(minimum(2)-3)+ (~(minimum(2)-3>1)).*1;
        moldy_max = (maximum(2)+3<640).*(maximum(2)+3) +(~(maximum(2)+3<640)).*640;
        
        moldura(moldx_min:moldx_max , moldy_min:moldy_max) = 1;
         
         
         % moldura((minimum(1)-3:maximum(1)+30),(minimum(2)-3):(maximum(2)+3))=1;
        
        deptharray_obj = deptharray1.*moldura;
        xyz1=get_xyzasus(deptharray_obj(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        
        rgbd=get_rgbd(xyz1,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
        
         I = single(rgb2gray(rgbd));
        %I1 = single(rgb2gray(im1));
        
        %[fa,da] = vl_sift(I1);
        %encontra features dentro de uma janela que envolve o objecto encontrado
        [ramiro(k).f,ramiro(k).d] = vl_sift(I);
        
        
   end
    par = zeros(size(frames_obj2,3),size(frames_obj1,3));
    p = 1;
    encontrado = [];
        % percorrer todos os conjuntos de pontos encontrados na c�mera 2

   for k = 1 :size(frames_obj2,3)
       if(isempty(frames_obj2))
           break;
       end
       [row,col]=find(frames_obj2(:,:,k)==1);
        
     	maximum = max([row';col']');
        minimum = min([row';col']');
        moldura = zeros(480,640);
        
        moldx_min = (minimum(1)-3>1).*(minimum(1)-3)+ (~(minimum(1)-3>1)).*1;
        moldx_max = (maximum(1)+30<480).*(maximum(1)+30) +(~(maximum(1)+30<480)).*480;
        moldy_min = (minimum(2)-3>1).*(minimum(2)-3)+ (~(minimum(2)-3>1)).*1;
        moldy_max = (maximum(2)+3<640).*(maximum(2)+3) +(~(maximum(2)+3<640)).*640;
        
        
       moldura(moldx_min:moldx_max , moldy_min:moldy_max) = 1;
        
        
        deptharray_obj = deptharray2.*moldura;
        xyz1=get_xyzasus(deptharray_obj(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        
        rgbd=get_rgbd(xyz1,im2,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
     
        
        I = single(rgb2gray(rgbd));
        
        %[fa,da] = vl_sift(I1);
        [gois(k).f,gois(k).d] = vl_sift(I);
        
      
        
        
        deptharray_obj = deptharray2.*frames_obj2(:,:,k);
        xyz1=get_xyzasus(deptharray_obj(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        
     
        
        rgbd=get_rgbd(xyz1,im2,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
        
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
            if(isempty(frames_obj1))
                  break;
            end
             %se o objecto vindo de 1 ja foi encontrado o seu par n�o se
             %vai fazer compara��o
           if(find(encontrado == j))
               continue;
           end
           %fazer match de feature dos dois conjuntos de pontos a ser analisados neste momento
          [matches,scores] = vl_ubcmatch(gois(k).d,ramiro(j).d);
          
          % vai ser atribu�do um scora a cada match camara1-camare2
          
          calc_score = length(matches);%n�mero de features encontradas nas duas janelas
          calc_score = calc_score + 10*log(abs((1/(gois(k).hue_score - ramiro(j).hue_score))));%fun��o que tem um valor tanto maior quanto menor for a diferen�a entre a tonalidade dos dois objcetos a comparar
          
          
          % se o score desta compara��o for superior ao melhor at� ao
          % momento ent�o quer dizer que este par tem maior probabilidade
          % de estar correcto que todas as compar��es feitas at� ao momento
          if (high_score < calc_score)
              %guarda o score actual
                high_score = calc_score;
                %guarda qual o par que tem o melhor score
                found = j;
                % guarda que objecto da imagem 1 j� tem o par para n�o ser comparado na pr�xima itera��o
                encontrado(p) = found;
              
          end
       end
           % a�s ter sido encontrado o par mais prov�vel para o objecto da
           % camera 2 vai-se avaliar se o score � sufucientemente alto para
           % considerar que existe uma correspondencia entre dois objectos
           % das duas projectivas
            if(high_score < 8 )
                %obj est� sozinho hehe
%                 objects(length(objects)+1).descriptor = d;%aqui vai ser inserido o primeiro objecto log nas linhas seguintes n�a vai ser preciso por o factor +1 nas linhas seguintes
%                 objects(length(objects)).X = [];
%                 objects(length(objects)).Y = [];
%                 objects(length(objects)).Z = [];
%             
%                 objects(length(objects)).frames_tracked = i;
% i belive this does not belong here
            else
                %est� nas duas imagens
                %EMPARELHATE
                par(k, found) = 1;
                  p = p+1;
               
            end
            
   
       
       
   
   end

   %procura se existe alguma coluna preenchida de 0 um objecto visto em 1
   %n�o foi encontrado par 
   B = any(par);
   p = 0;
        close all  
   
   % percorrer todos os conjuntos de pontos encontrados na c�mera 2
   for n=1:size(frames_obj2,3)
        if(isempty(frames_obj2))
           break;
       end
           % percorrer todos os conjuntos de pontos encontrados na c�mera 1
    for m=1:size(frames_obj1,3)
         if(isempty(frames_obj1))
           break;
         end
         %se foi encontrado uma correspondecia entre duas objectos em
         %camaras diferentes vai-se efectuar a liga��o
        if(par(n,m)==1)
              p = p +1 ;
              % em miragaia vai guardar um conjun��o dos blocos de pontos que lhe pertencem 
            miragaia(p).d = [ramiro(m).d , gois(n).d];  %concatena as features encontradas
            miragaia(p).hue_score = (ramiro(m).hue_score + gois(n).hue_score)/2;% faz a m�dia da tonalidade
          
        %emparelha
         deptharray_obj1 = deptharray1.*frames_obj1(:,:,m);
          deptharray_obj2 = deptharray2.*frames_obj2(:,:,n);
           
        xyz1=get_xyzasus(deptharray_obj1(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        xyz2=get_xyzasus(deptharray_obj2(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        
        %elimina zeros dos pontos 3d
        xyz1(  all(~xyz1,2), :  ) = [];
        xyz2(  all(~xyz2,2), :  ) = [];

        %faz point clouds dos foregrounds e faz merge destes
        xyz1inW = xyz1 ;       
        xyz2inW = xyz2*tr.T + ones(length(xyz2),1)*tr.c(1,:);
          
        pcFG1=pointCloud(xyz1inW);
        pcFG2=pointCloud(xyz2inW);

        
        FG_pts=pcmerge(pcFG1,pcFG2,0.001);
        
        %encontra os pontos m�ximos em 3D que v�o conter o objecto encontrado
        exrtremes1 = max(FG_pts.Location);
        minions1 = min(FG_pts.Location);
       % calcula os 8 pontos que representam o paralelipipedo que comtem o objecot encontrado         
       miragaia(p).X = [minions1(1),minions1(1),minions1(1),minions1(1), exrtremes1(1) ,exrtremes1(1) ,exrtremes1(1) ,exrtremes1(1)];
        
       miragaia(p).Y = [minions1(2),minions1(2), exrtremes1(2) ,exrtremes1(2) ,minions1(2),minions1(2),exrtremes1(2) ,exrtremes1(2)];
        
       miragaia(p).Z = [minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3)];
        
        %se entrar aqui quer dizer que foi encontrado o par fo objecto
        %encontrado em 2 por isso pode-se passar para o proximo objecto a
        %ser emparelhadp
        break;   
        
        end                
   
    end
    
    % se nesta combina��o n,m foi encontrado podemos ir analizar outro objecto
    if(par(n,m)==1)
        continue;
    end
    
    % se chegar aqui significa que n�o foi encontrado par para um objecto
    % encontrado na c�mara 2 por isso vai ser guardado na estrututra que
    % corresponde diretamente a um e apenas um objecto (miragaia)
           deptharray_obj2 = deptharray2.*frames_obj2(:,:,n);
            p = p +1 ;
            %objecto novo n�o � preciso concatenar, vai guadas todos os
            %valores
             miragaia(p).d =  gois(n).d; 
             miragaia(p).hue_score = gois(n).hue_score;
           
        xyz2=get_xyzasus(deptharray_obj2(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        
        %elimina zeros dos pontos 3d
    
        xyz2(  all(~xyz2,2), :  ) = [];

        %faz point clouds dos foregrounds e faz merge destes       
           
        xyz2inW = xyz2*tr.T + ones(length(xyz2),1)*tr.c(1,:);
        
 
        pcFG2=pointCloud(xyz2inW);

        
        FG_pts = pcFG2;

        
        exrtremes1 = max(FG_pts.Location);
        minions1 = min(FG_pts.Location);
        
        miragaia(p).X = [minions1(1),minions1(1),minions1(1),minions1(1), exrtremes1(1) ,exrtremes1(1) ,exrtremes1(1) ,exrtremes1(1)];
       miragaia(p).Y = [minions1(2),minions1(2), exrtremes1(2) ,exrtremes1(2) ,minions1(2),minions1(2),exrtremes1(2) ,exrtremes1(2)];
       miragaia(p).Z = [minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3)];
     
    
    
   end
   
    %v� se matriz par tem colunas a zero ou seja um objecto encontrado em 1
    %n�o tem um par correspondetne
    
    if(~isempty(frames_obj1))
    for n=1:size(frames_obj1,3)
        if(length(B) == 1)
            B = par;
        end
    if B(n) == 0
        p = p +1 ;
       
              miragaia(p).d = ramiro(n).d ;  
              miragaia(p).hue_score = ramiro(n).hue_score;
            
        deptharray_obj1 = deptharray1.*frames_obj1(:,:,n);
  
           
        xyz1=get_xyzasus(deptharray_obj1(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);

        
        %elimina zeros dos pontos 3d
        xyz1(  all(~xyz1,2), :  ) = [];
        xyz1inW = xyz1 ;              
       
        %faz point clouds dos foregrounds e faz merge destes
        pcFG1=pointCloud(xyz1inW);

        FG_pts=pcFG1;

         
        exrtremes1 = max(FG_pts.Location);
        minions1 = min(FG_pts.Location);        
       miragaia(p).X = [minions1(1),minions1(1),minions1(1),minions1(1), exrtremes1(1) ,exrtremes1(1) ,exrtremes1(1) ,exrtremes1(1)];
        
       miragaia(p).Y = [minions1(2),minions1(2), exrtremes1(2) ,exrtremes1(2) ,minions1(2),minions1(2),exrtremes1(2) ,exrtremes1(2)];
        
       miragaia(p).Z = [minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3) ,minions1(3) ,exrtremes1(3)];        
    
    end
    
    end
       
    end

    
    
%codigo relevante tracking
% aqui faz-se a correspondencia entre frames anteriores com o frames
% actual, se n�o existe nenhum objecto v�o ser inserido diretamente o que
% foram encontrados neste instante
if(~isempty(objects)) 

encontrado = [];
    p =1;
    %percorrer por todos os objectos encontrados neste frame
    for m=1:length(miragaia)
        
        num_matches = 0;
        %percorrer pelos objectos que foram encontrados no frame anterior
        for n=1:length(miragaia_prev)
          
            %se o objecto anterior j� foi emparelhado com um dos objectos
            %atuais n�o ser tentado ser emparelhado
             if(find(encontrado == n))
                 continue;
             end
          
             %o score � calculado como no case do mesmo frame
            [matches,~] = vl_ubcmatch(miragaia_prev(n).d,miragaia(m).d);
      
             calc_score = length(matches);
          
            calc_score = calc_score + 10*log(abs((1/(miragaia(m).hue_score - miragaia_prev(n).hue_score))));
            
            if num_matches < calc_score
                num_matches = calc_score;
             found = n; %found e o antigo
             encontrado(p) = found;
             
             end  
        end
        %neste caso o threshold inferior � superior pois agora existe uma maior base de dados e que seja maiores os resultados de matching 
            if num_matches > 30
              %adicionar ao obj m
                objects(miragaia_prev(found).obj_prev).X=[objects(miragaia_prev(found).obj_prev).X ; miragaia(m).X];
                objects(miragaia_prev(found).obj_prev).Y=[objects(miragaia_prev(found).obj_prev).Y ; miragaia(m).Y];
                objects(miragaia_prev(found).obj_prev).Z=[objects(miragaia_prev(found).obj_prev).Z ; miragaia(m).Z];
                objects(miragaia_prev(found).obj_prev).frames_tracked = [objects(miragaia_prev(found).obj_prev).frames_tracked,i];
                %guardar a que objecto pertence
                miragaia(m).obj_prev = miragaia_prev(found).obj_prev;
                p = p+1;
                
            else 
                %se n�o for encontrado um par quer dizer que nos
                %encontramos perante um novo objecto por isso deve ser
                %adicionado
                objects(length(objects)+1).frames_tracked = i;
                %ja adicionamos ja fica crto
                objects(length(objects)).X = miragaia(m).X;
                objects(length(objects)).Y = miragaia(m).Y;
                objects(length(objects)).Z = miragaia(m).Z;
                miragaia(m).obj_prev = length(objects);
            end
    end
 %caso de ainda n�o existirem objectos    
else
  %primeira vez
  
  objects=miragaia;
  
  for t = 1:size(miragaia,2)
  miragaia(t).obj_prev = t;
  objects(t).frames_tracked = i;
  end
  
end
    



    


    
%guardas as caracteristicas dos objectos para serem comparados no pr�ximo
%frame
    miragaia_prev = miragaia;

    
end

%definir os vari�veis de sa�da 
% como foi escolhido a camara 1 como origim a matriz de rota��o vai ser a
% identidade e a transla��o ser� nula
cam1toW.R = eye(3)';
cam1toW.T = zeros(1,3)';

% as matrizes de transforma��o ser�o as c�lculadas pelo procrustes
cam2toW.R = tr.T';
cam2toW.T = tr.c(1,:)';



end

