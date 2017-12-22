function [tr] = procrustesverdadeiro(file_name,Depth_cam)
%% threshold  pode estar mal<---------- <---------
% READ IMAGES and GENERATE POINT CLOUDS



%file_name='maizena'
load ../matlab.mat;

%%


%faz load das imagens de profundidade e depth
im1=imread(['../' file_name '/rgb_image1_1.png']);
im2=imread(['../' file_name '/rgb_image2_1.png']);
load(['../' file_name '/depth1_1.mat'])
dep1=depth_array;
load(['../' file_name '/depth2_1.mat'])
dep2=depth_array;

%converte em coodenadas 3D ISTO E SO DEBUG E PARA REMOVER TUDO
% xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
% xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
% 
% %REGISTER DEPTH to RGB
% rgbd1 = get_rgbd(xyz1, im1, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
% rgbd2 = get_rgbd(xyz2, im2, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
% figure(1);imagesc(rgbd1 );
% figure(2);imagesc(rgbd2 );
% 
% pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
% pc2=pointCloud(xyz2,'Color',reshape(rgbd2,[480*640 3]));
% figure(3); showPointCloud(pc1);
% figure(4); showPointCloud(pc2);

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
        [matches, scores] = vl_ubcmatch(d1,d2);
         
        %converte os indices de 2D para array 
        ind1=sub2ind(size(dep1),round(f1(2,matches(1,:))),round(f1(1,matches(1,:))));
        ind2=sub2ind(size(dep2),round(f2(2,matches(2,:))),round(f2(1,matches(2,:))));
         
%          pts2 = round(f2(1:2,matches(2,:)));
%          pt2 = pts2(1,:)+(pts2(2,:)-1)*640;
        %busca pontos 3D importantes de todos os pontos 3D
%         xyz_pts1 = xyz1(pt1,:);
       %  xyz_pts2 = xyz2(pt2,:);
         
       
       
         %apenas usa pontos emparelhados
         xyz_pts1=xyz1(ind1,:);
         xyz_pts2=xyz2(ind2,:);
         
         %inicializa highscore
         high_score_inliers = 0;
         
         %itera x vezes
         for i=1:4000            
             
             %escolhe 4 pontos
             perm = randperm(size(matches,2),4) ;

             %vai buscar os 4 pontos em cada camara
             P1 = xyz_pts1(perm,:);
             P2 = xyz_pts2(perm,:);

             %resolve o problema do procrustes
             [d,xx,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);

             %converte as coordenadas do referencial da camara 2 para a camara
             %1
             xyz2in1=xyz_pts2*tr.T+ones(length(xyz_pts2),1)*tr.c(1,:);

             %calcula a distancia euclideana entre o ponto em 1 e a sua
             %transformacao
             temp = xyz2in1 - xyz_pts1;
             norms = sqrt(sum(temp'.^2,1));

             %verifica se estam proximos
             inliers_id = find( norms < 0.10); %PODE NAO ESTAR BEM, ATENÇÃO RAMIRAGOIS

             %define esta transformacao como sendo a melhor caso tenha mais
             %inliers
             if(length(inliers_id) > high_score_inliers)

                  high_score_inliers = length(inliers_id);
                  best_inliers = inliers_id;

              end

    %     pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
    %     pc2=pointCloud(xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:),'Color',reshape(rgbd2,[480*640 3]));
    %     close all
    %     figure
    %     %showPointCloud(pc1)
    %     pcshow(pcmerge(pc1,pc2,0.001));


         end
         
       %este debug pode ser apagado  
        %im1
        deptharray_obj = dep1;
        xyz1=get_xyzasus(deptharray_obj(:),[480 640],(1:640*480)',Depth_cam.K,1,0);
        rgbd1=get_rgbd(xyz1,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
        
     
        %im2
        deptharray_obj = dep2;
        xyz2=get_xyzasus(deptharray_obj(:),[480 640],(1:640*480)',Depth_cam.K,1,0);      
        rgbd2=get_rgbd(xyz2,im2,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
    pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
    pc2=pointCloud(xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:),'Color',reshape(rgbd2,[480*640 3]));
    close all
    figure
    pcshow(pcmerge(pc1,pc2,0.001));
         

          
         %vai buscar todos os bons inliers a ambos os vetores
         perm = best_inliers;
         P1 = xyz_pts1(perm,:);
         P2 = xyz_pts2(perm,:);
        
         
         %vai buscar todos os bons inliers a ambos os vetores e calcula o
         %procrustes
         %P1=P1(inds,:);
       %  P2=P2(inds,:);
         [~,~,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);
         
         
        
         
         
    %debug
    pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
    pc2=pointCloud(xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:),'Color',reshape(rgbd2,[480*640 3]));
    close all
    figure
    %showPointCloud(pc1)
    pcshow(pcmerge(pc1,pc2,0.001));
    drawnow;


        
end
        
        
        
        
        
