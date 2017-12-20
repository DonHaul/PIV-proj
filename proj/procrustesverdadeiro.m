%% threshold  pode estar mal<---------- <---------
% READ IMAGES and GENERATE POINT CLOUDS
load ../imagens/matlab.mat;
im1=imread('../room/rgb_image1_3.png');
im2=imread('../room/rgb_image2_3.png');
load('../room/depth1_3.mat')
dep1=depth_array;
load('../room/depth2_3.mat')
dep2=depth_array;
%dep2(find(dep2(:)>4000))=0;
xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
%REGISTER RGB TO DEPTH
rgbd1 = get_rgbd(xyz1, im1, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
rgbd2 = get_rgbd(xyz2, im2, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
figure(1);imagesc(rgbd1 );
figure(2);imagesc(rgbd2 );



pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
pc2=pointCloud(xyz2,'Color',reshape(rgbd2,[480*640 3]));
figure(3); showPointCloud(pc1);
figure(4); showPointCloud(pc2);

%%
%GET CORRESPONDING POINTS


%SIFT
   deptharray_obj = dep1;
        xyz1=get_xyzasus(deptharray_obj(:),[480 640],(1:640*480)',Depth_cam.K,1,0);
        
        rgbd1=get_rgbd(xyz1,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
        
         I1 = single(rgb2gray(im1));
       [f1,d1] = vl_sift(I1) ;
        
%im2
  deptharray_obj = dep2;
        xyz2=get_xyzasus(deptharray_obj(:),[480 640],(1:640*480)',Depth_cam.K,1,0);
        %ATENCAO APAZ RGB VIRTUal e nao o DA IMAGEM BOA
        %         
        rgbd2=get_rgbd(xyz2,im2,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
        
         I2 = single(rgb2gray(im2));
       [f2,d2] =  vl_sift(I2);
         [matches, scores] = vl_ubcmatch(d1,d2);
         
         %obtem posições xy e arredonda 
%          pts1 = round(f1(1:2,matches(1,:)));
         %converte de xy para array/indice
%          pt1 = pts1(1,:)+(pts1(2,:)-1)*640;
         
         
         ind1=sub2ind(size(dep1),round(f1(2,matches(1,:))),round(f1(1,matches(1,:))));
         ind2=sub2ind(size(dep2),round(f2(2,matches(2,:))),round(f2(1,matches(2,:))));
         
%          pts2 = round(f2(1:2,matches(2,:)));
%          pt2 = pts2(1,:)+(pts2(2,:)-1)*640;
        %busca pontos 3D importantes de todos os pontos 3D
%         xyz_pts1 = xyz1(pt1,:);
       %  xyz_pts2 = xyz2(pt2,:);
         
         xyz_pts1=xyz1(ind1,:);
         xyz_pts2=xyz2(ind2,:);
         high_score_inliers = 0;
         
         for i=1:4000
            
             
         perm = randperm(size(matches,2),6) ;
         P1 = xyz_pts1(perm,:);
         P2 = xyz_pts2(perm,:);
         [d,xx,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);
         
         

         
         
          xyz2in1=xyz_pts2*tr.T+ones(length(xyz_pts2),1)*tr.c(1,:);
          
          temp = xyz2in1 - xyz_pts1;
          norms = sqrt(sum(temp'.^2,1));
          
         inliers_id = find( norms < 0.10); %PODE NAO ESTAR BEM, ATENÇÃO RAMIRAGOIS
          
          if(length(inliers_id) > high_score_inliers)
              
              high_score_inliers = length(inliers_id);
              best_inliers = inliers_id;
              
          end
          
%            pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
%     pc2=pointCloud(xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:),'Color',reshape(rgbd2,[480*640 3]));
%     close all
%     figure
%     %showPointCloud(pc1)
%     pcshow(pcmerge(pc1,pc2,0.001));


         end
         
         
         
         %%
         
         
         
        %  load ../maizena/rly_close.mat;
         
         
         
            deptharray_obj = dep1;
        xyz1=get_xyzasus(deptharray_obj(:),[480 640],(1:640*480)',Depth_cam.K,1,0);
        
        rgbd1=get_rgbd(xyz1,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
        
     
%im2
  deptharray_obj = dep2;
        xyz2=get_xyzasus(deptharray_obj(:),[480 640],(1:640*480)',Depth_cam.K,1,0);
        %ATENCAO APAZ RGB VIRTUal e nao o DA IMAGEM BOA
        %         
        rgbd2=get_rgbd(xyz2,im2,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
                    pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
    pc2=pointCloud(xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:),'Color',reshape(rgbd2,[480*640 3]));
    close all
    figure
    %showPointCloud(pc1)
    pcshow(pcmerge(pc1,pc2,0.001));
         
         
          xyz2in1=xyz_pts2*tr.T+ones(length(xyz_pts2),1)*tr.c(1,:);
          
          temp = xyz2in1 - xyz_pts1;
          norms = sqrt(sum(temp'.^2,1));
          
         
         perm = best_inliers;
          P1 = xyz_pts1(perm,:);
         P2 = xyz_pts2(perm,:);

           P1=P1(inds,:);
           P2=P2(inds,:);
         [d,xx,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);
         
         
        
         
         
        
    pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
    pc2=pointCloud(xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:),'Color',reshape(rgbd2,[480*640 3]));
    close all
    figure
    %showPointCloud(pc1)
    pcshow(pcmerge(pc1,pc2,0.001));
    drawnow;


        
        
        
        
        
        
        
