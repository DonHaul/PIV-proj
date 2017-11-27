function [ r,c, flag,size] = getextremes_depth( imagem, backGround )
%GETEXTREMES_DEPTH Summary of this function goes here
%   Detailed explanation goes here

    %flag que contem a informa��o se existe algum objecto na imagem a ser
    %analisada
    imagemCamera = imagem;
    flag = 0;
    c= [];
    r = [];
    size = [];
    % subtra� o backgroudn � imagem a ser analizada para obter apenas 
    imagem = abs(double(imagem) - backGround)>0.25;%erro de 25 cm da camera de profundidade
    
    RED = edge(imagem,'Sobel');
    for i =1:480
        for j=1:640
            if RED(i,j)== 1
                imagem(i,j)=0;
            end
        end
    end
    % agrupa os pixeis em vizinha�a 8 ,dando um valor diferente de label
    % para cada conjunto
    imagem = bwlabel(imagem);
    
    
    % guarda o valor em forma de matriz da imagem
    TEMP = imagem;
    
   
    imagem = squeeze(imagem);
    imagesc(imagem);
    
    %separa os diversps conjuntos e faz a sua ordena��o
    GMI = imagem(:);
    %separa todos os conjuntos de pixeis que foram encontrados 
    [n,~] = hist(GMI(:),unique(GMI(:)));
    %ordena os conjuntos de pixeis pelo seu tamanho
    [~,idx] = sort(-n);

   
    pos_M_Y=0;
    pos_m_Y = 0;
    pos_m_X = 0 ;
    pos_M_X = 0;
    
    % por todos os conjuntos com mais de 3000 pixeis (1% de uma imagens)
    % pode/deve ser modificado para outro valor
    
     % o maior � o brackbground por isso so se come�a a contar  depois deste
    j=2;
    
    while( n(idx(j)) > 1E3) % por todos os conjuntos com mais de 3000 pixeis (1% de uma imagens), pode/deve ser modificado para outro valor


        %guarda nas vari�veis de ouput o conjunto linhas/colunas que pertecen � label que est� a ser analisada agora
        [a, b] = find(TEMP==(idx(j)-1));%procura todos os pixeis que pertencem ao conjunto que presentemente est� a ser analisado
        
        c = [c;a];
        r = [r;b];
        size = [size;n(idx(j))];
        %vari�veis que guardam os extremos em 2D na imagem de profundidade
        % pos_ guardo o valor do extremo
        % idx_ guarda a posi��o do extremo
        [pos_M_Y,idx_M_Y] = max(r);
        [pos_M_X,idx_M_X] = max(c);
        [pos_m_Y,idx_m_Y] = min(r);
        [pos_m_X,idx_m_X] = min(c);
        
        
        j=j+1;
        
       %assinala que foi encontrado objst�culo
        flag = flag +1 ;
    end
    
    % se n�o foi encotrado nenhum objecto sai da fun��o
    if flag == 0
        return
    end
    

    hold on
    %plot para verificar os pontos maximos/minimos do objecto
    % v�o ser representados na imagem do foreground que come�ou a ser exibida no programa principal 
    % delimitando o objecto encontrado, sendo que segundo esta representa��o todos os pontos do ojecto estar�o contidos dentro do rectangulo delimitante
    scatter(pos_m_X,pos_M_Y,'X')
    scatter(pos_m_X,pos_m_Y,'X')
    scatter(pos_M_X,pos_M_Y,'X')
    scatter(pos_M_X,pos_m_Y,'X')
    
   % pause(1)
    % pontos a possivelmente ser analisdos 
    % estes pontos correspondem ao ponto que possui uma das coordenada
    % m�ximas/m�nimas que pertence ao objecto
    extremos = [ c(idx_M_X),r(idx_M_X) ;
     c(idx_m_X),r(idx_m_X) ;
     c(idx_M_Y),r(idx_M_Y) ;
     c(idx_m_Y),r(idx_m_Y) ;];  

end

