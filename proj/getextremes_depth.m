function [ r,c, flag] = getextremes_depth( imagem, backGround )
%GETEXTREMES_DEPTH Summary of this function goes here
%   Detailed explanation goes here

    %flag que contem a informação se existe algum objecto na imagem a ser
    %analisada
    flag = 0;
    % subtraí o backgroudn à imagem a ser analizada para obter apenas 
    imagem = abs(double(imagem) - backGround)>0.25;%erro de 25 cm da camera de profundidade
    
    % agrupa os pixeis em vizinhaça 8 ,dando um valor diferente de label
    % para cada conjunto
    imagem = bwlabel(imagem);
    
    % guarda o valor em forma de matriz da imagem
    TEMP = imagem;
    
    
    imagem = squeeze(imagem);
    imagesc(imagem);
    
    %separa os diversps conjuntos e faz a sua ordenação
    GMI = imagem(:);
    [n,~] = hist(GMI(:),unique(GMI(:)));
    [~,idx] = sort(-n);
    % o maior é o brackbground por isso so se começa a contar  depois deste
    j=2;
    
    pos_M_Y=0;
    pos_m_Y = 0;
    pos_m_X = 0 ;
    pos_M_X = 0;
    
    % por todos os conjuntos com mais de 3000 pixeis (1% de uma imagens)
    % pode/deve ser modificado para outro valor
    
    while( n(idx(j)) > 3E3)
        %guarda nos valores que vão ser retornados o conjunto
        %linhas/colunas que pertecen à label que está a ser analisada agora
        [r, c] = find(TEMP==(idx(j)-1));
        
        
        %variáveis que guardam os extremos em 2D na imagem de profundidade
        [pos_M_Y,idx_M_Y] = max(r);
        [pos_M_X,idx_M_X] = max(c);
        [pos_m_Y,idx_m_Y] = min(r);
        [pos_m_X,idx_m_X] = min(c);
        
        
        j=j+1;
        
       
        flag = 1;
    end
    
        if flag == 0
            return
        end
    
    hold on
    %plot para verificar os pontos maximos/minimos do objecto
    scatter(pos_m_X,pos_M_Y,'X')
    scatter(pos_m_X,pos_m_Y,'X')
    scatter(pos_M_X,pos_M_Y,'X')
    scatter(pos_M_X,pos_m_Y,'X')
    
    pause(1)
    % pontos a possivelmente ser analisdos 
    % estes pontos correspondem ao ponto que possui uma das coordenada
    % máximas/mínimas que pertence ao objecto
    extremos = [ c(idx_M_X),r(idx_M_X) ;
     c(idx_m_X),r(idx_m_X) ;
     c(idx_M_Y),r(idx_M_Y) ;
     c(idx_m_Y),r(idx_m_Y) ;];  

end

