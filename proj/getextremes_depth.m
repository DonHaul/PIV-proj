function [ extremos] = getextremes_depth( imagem, backGround )
%GETEXTREMES_DEPTH Summary of this function goes here
%   Detailed explanation goes here
    imagem = abs(imagem - backGround)>320;
    
    imagem = bwlabel(imagem);
    TEMP = imagem;
    
    imagem = squeeze(imagem);
    imagesc(imagem);
   
    colormap(gray);
    
    
    GMI = imagem(:);
    [n,bin] = hist(GMI(:),unique(GMI(:)));
    [~,idx] = sort(-n);
    % o maior é o brackbground
    j=2;
    % por todos os conjuntos com mais de 3000 pixeis (1% de uma imagens)
    while( n(idx(j)) > 10E3)
        [r, c] = find(TEMP==(idx(j)-1));
        pos_M_Y = max(r);
        pos_M_X = max(c);
        pos_m_Y = min(r);
        pos_m_X = min (c);
        
        
        j=j+1;
    
    end
        hold on
        scatter(pos_m_X,pos_M_Y,'X')
        scatter(pos_m_X,pos_m_Y,'X')
        scatter(pos_M_X,pos_M_Y,'X')
        scatter(pos_M_X,pos_m_Y,'X')
    pause(1)
extemos = [pos_m_X pos_M_X pos_m_Y pos_M_Y];

end

