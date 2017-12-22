function [depthArrayFG1,depthArrayFG2,frame_obj1, frame_obj2] = getForeGroundpts( backGround_a,backGround_b,deptharray1,deptharray2)
% encontra conjuntos de pontos correspondentes a objectos pertencentes ao foreground em cada uma das c�meras
% [depthArrayFG1,depthArrayFG2,frame_obj1, frame_obj2] = getForeGroundpts( backGround_a,backGround_b,deptharray1,deptharray2)
% output
%   depthArrayFGi conjunto de imagens em que cada elemeto � composto por  pela imagem de profundidade do objecto correspondente
%           i = 1,2
%    frame_obji conjunto de imagens binarias com valor de 1 nos pixeis que correspondem a 1 objecto encontrado
%           i =1,2
% inputs
%   backGround_i imagem de background de profundidade da prespectiva i
%           i = a,b
%   deptharrayi imagem de profundidade a ser analisada da prespectiva i
%           i = 1,2

depthArrayFG1= [];
depthArrayFG2= [];
frame_obj1= [];
frame_obj2 = [];
    
    %background subtraction (valor de cada pixel vai assumir 1 ou 0 consante a condi��o apresentada)
    %neste caso um pixel pertence ao background se a sua dist�ncia � imagem
    %de background for superior a 20cm
    fg1 = abs(double(deptharray1) - backGround_a) > 0.2;
    fg2 = abs(double(deptharray2) - backGround_b)> 0.2; %sem abs
    
    
    %elimina os zeros do foreground (foreground fica com fator de escala mas como esta fun��o apenas necessita saber se um ponto pertence ao foreground ou n�o)
    fg1= fg1 .*double(deptharray1);
    fg2= fg2 .*double(deptharray2);

     
     %%
     % Para que seja possivel distinguir melhor os contornos dos objecto �
     % aplicado im filtro de gradiente
     
     [Gmag,Gdir] = imgradient(fg1);% magnitude e direc��o do gradiente de cada uma das imagens
     Gmag(Gmag<0.3)=0;%remove os valores muito baixos do gradiente para que altera��es insignificantes sejam ignorados
     fg1 = fg1.*~Gmag;% locais de elevado gradiente v�o passar a ter 0, causando que os considremos como background, logo as edges de cada objecto localizado na imagem sejam identificados.
     [Gmag,Gdir] = imgradient(fg2);
     Gmag(Gmag<0.3)=0;
     fg2 = fg2.*~Gmag;
    %%
    %faz black and white label
    %agrupa os pontos com vizinha�a 8 mediante pertencerem ao background ou
    %foreground
    bw1 = bwlabel(fg1,8);
    bw2 = bwlabel(fg2,8);

    %ordena os conjuntos pelo seu tamanho
    labelCounts1=tabulate(bw1(:));
    labelCounts2=tabulate(bw2(:));
   
    %devolve o indice do label e consequentemente o label
    goodLabels1 =find(labelCounts1(:,2)>500)-1;
    goodLabels2 =find(labelCounts2(:,2)>500)-1;
    
    %removes bg from goodlabels encontra o max das counts e remove o dos
    %good labels 
    % o background corresponde ao maior conjunto de pontos, por isso v�o se
    % considerar todos os outros conjuntos
    goodLabels2(find(max(labelCounts2(:,2))))=[];
    goodLabels1(find(max(labelCounts1(:,2))))=[];

  %gera imagem onde 1 corresponde aos  pixeis dos objetos , os outros pixeis v�o ter valor 0
  goodItems1=zeros(480,640);
  frame_obj1=zeros(480,640,length(goodLabels1));
  %Separar os conjuntos de pontos em "imagens" separadas
  for i=1:length(goodLabels1)
  goodItems1=goodItems1 | bw1==goodLabels1(i);
  frame_obj1(:,:,i) =  bw1==goodLabels1(i);
  end


  goodItems2=zeros(480,640);
  %para a segunda c�mera
  for i=1:length(goodLabels2)
  goodItems2=goodItems2 | bw2==goodLabels2(i);
  frame_obj2(:,:,i) =  bw2==goodLabels2(i);
  end



%vai buscar ao depth array apenas os objetos bons
%imagens de profundidade dos objectos encontrados
depthArrayFG1= deptharray1.*goodItems1;
depthArrayFG2= deptharray2.*goodItems2;





end

