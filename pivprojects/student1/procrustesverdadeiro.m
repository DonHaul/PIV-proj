function [tr] = procrustesverdadeiro(cam1,cam2,Depth_cam,R_d_to_rgb,T_d_to_rgb,RGB_cam)
% função que a partir de duas imagens de cor e profundidade vai encontrar a transformação entre os referenciais das câmeras, retornado uma estrutura com a matriz de rotação e tranlação da câmera 2 para a 1
% [tr] = procrustesverdadeiro(cam1,cam2,Depth_cam,R_d_to_rgb,T_d_to_rgb,RGB_cam)
% output
% tr -> tr.R matriz de rotação entre a 2 e 1
%    -> tr.T vector de translação de 2 para 1
% input
% cami -> cami.rgb localização da imagem de cor
%      -> cami.depth localização da imagem de profundidade
% para i = 1,2
% Depth_cam -> Depth_cam.K matriz com os parametros intrinsecos da câmera de profundidade
% R_d_to_rgb -> matriz com parametros extrinsecos para transformar da
% câmera de profundidade para a de cor(rotação)
% T_d_to_rgb -> vector com parametros extrinsecos para transformar da
% câmera de profundidade para a de cor(translação)
% RGB_cam -> RGB_cam.K matriz com os parametros intrinsecos da câmera de cor


%%


%faz load das imagens de profundidade e depth
im1=imread(cam1.rgb);
im2=imread(cam2.rgb);
load(cam1.depth)
dep1=depth_array;
load(cam2.depth)
dep2=depth_array;

%%
%SIFT

%Faz o sift da imagem de rgb retificada para depth da camara 1
deptharray_obj = dep1;
xyz1=get_xyzasus(deptharray_obj(:),[480 640],(1:640*480)',Depth_cam.K,1,0);
rgbd1=get_rgbd(xyz1,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);

I1 = single(rgb2gray(im1));
[f1,d1] = vl_sift(I1) ;

%Faz o sift da imagem de rgb retificada para depth da camara 2
deptharray_obj = dep2;
xyz2=get_xyzasus(deptharray_obj(:),[480 640],(1:640*480)',Depth_cam.K,1,0);
rgbd2=get_rgbd(xyz2,im2,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);

I2 = single(rgb2gray(im2));
[f2,d2] =  vl_sift(I2);

%emparelha os keypoints
[matches, ~] = vl_ubcmatch(d1,d2);

%converte os indices de 2D para array
ind1=sub2ind(size(dep1),round(f1(2,matches(1,:))),round(f1(1,matches(1,:))));
ind2=sub2ind(size(dep2),round(f2(2,matches(2,:))),round(f2(1,matches(2,:))));



%apenas usa pontos emparelhados
xyz_pts1=xyz1(ind1,:);
xyz_pts2=xyz2(ind2,:);

%inicializa highscore
high_score_inliers = 0;

%itera x vezes
for i=1:4000
    
    %escolhe 4 pontos, numero mínimo de pontos para que possa ser
    %resolvido o procruestes
    perm = randperm(size(matches,2),4) ;
    
    %vai buscar os 4 pontos em cada camara
    P1 = xyz_pts1(perm,:);
    P2 = xyz_pts2(perm,:);
    
    %resolve o problema do procrustes com os pares de pontos
    %escolhidos nesta iteração
    [~,~,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);
    
    %converte as coordenadas do referencial da camara 2 para a camara 1
    xyz2in1=xyz_pts2*tr.T+ones(length(xyz_pts2),1)*tr.c(1,:);
    
    %calcula a distancia euclideana entre o ponto em 1 e a sua
    %transformacao
    temp = xyz2in1 - xyz_pts1;
    norms = sqrt(sum(temp'.^2,1));
    
    %verifica se estam proximos
    % guarda os pontos que a sua distancia entre a sua
    % posição real e a sua projecção seja inferior a 10cm
    inliers_id = find( norms < 0.10);
    
    %define esta transformacao como sendo a melhor caso um maior
    %número de inliers
    if(length(inliers_id) > high_score_inliers)
        
        %guarda o número de inliers encontrados resultantes da
        %transformação gerado pelos pontos escolhidos
        high_score_inliers = length(inliers_id);
        %guarda os indices dos inliers para serem usados
        %posteriormente
        best_inliers = inliers_id;
        
    end
    
end

%vai buscar todos os bons inliers de ambos os vetores
%correspondetes ao conjunto em que foram encontrados mais inliers
perm = best_inliers;
P1 = xyz_pts1(perm,:);
P2 = xyz_pts2(perm,:);


[~,~,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);





end





