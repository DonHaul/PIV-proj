function [ r,c, flag] = getextremes_depth( imagem, backGround )
%GETEXTREMES_DEPTH Summary of this function goes here
%   Detailed explanation goes here
    r=0;
    c=0;
    flag = 0;
    extremos = zeros(4,2);
    depth = imagem;
    imagem = abs(double(imagem) - backGround)>0.1;%erro de 35 cm da camera
    imagem = bwlabel(imagem);
    TEMP = imagem;
    
    
    imagem = squeeze(imagem);
    imagesc(imagem);
    
    GMI = imagem(:);
    [n,~] = hist(GMI(:),unique(GMI(:)));
    [~,idx] = sort(-n);
    % o maior � o brackbground
    j=2;
    % por todos os conjuntos com mais de 3000 pixeis (1% de uma imagens)
    pos_M_Y=0;
    pos_m_Y = 0;
    pos_m_X = 0 ;
    pos_M_X = 0;
    
       
    while( n(idx(j)) > 3E3)
        %de momento so suporta um objecto
        [r, c] = find(TEMP==(idx(j)-1));
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
    
    scatter(pos_m_X,pos_M_Y,'X')
    scatter(pos_m_X,pos_m_Y,'X')
    scatter(pos_M_X,pos_M_Y,'X')
    scatter(pos_M_X,pos_m_Y,'X')
    
    pause(1)
    extremos = [ c(idx_M_X),r(idx_M_X) ;
     c(idx_m_X),r(idx_m_X) ;
     c(idx_M_Y),r(idx_M_Y) ;
     c(idx_m_Y),r(idx_m_Y) ;];  

end

