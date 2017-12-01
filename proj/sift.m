    im1 = imread([myDir '1_' int2str(9) ext_img]);
    im2 = imread([myDir '2_' int2str(9) ext_img]);
    
    I = single(rgb2gray(im1));
    I2 = single(rgb2gray(im2));
    
    [fa,da] = vl_sift(I) ;
    [fb,db] = vl_sift(I2) ;
    [matches, scores] = vl_ubcmatch(da, db) ;

%     perm = randperm(size(f,2)) ;
%     sel = perm(1:50) ;
%     h1 = vl_plotframe(f(:,sel)) ;
%     h2 = vl_plotframe(f(:,sel)) ;
%     set(h1,'color','k','linewidth',3) ;
%     set(h2,'color','y','linewidth',2) ;
%     h3 = vl_plotsiftdescriptor(d(:,sel),f(:,sel)) ;
%     set(h3,'color','g') ;
%     
    
    
  goodlabels1
    
    
    
    
    
    
    
    
    
    
    