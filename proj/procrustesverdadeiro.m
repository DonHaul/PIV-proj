%%
% READ IMAGES and GENERATE POINT CLOUDS
load ../imagens/matlab.mat;
im1=imread('../maizena/rgb_image1_3.png');
im2=imread('../maizena/rgb_image2_3.png');
load('../maizena/depth1_3.mat')
dep1=depth_array;
load('../maizena/depth2_3.mat')
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
        xyz1=get_xyzasus(deptharray_obj(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        
        rgbd1=get_rgbd(xyz1,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
        
         I1 = single(rgb2gray(im1));
       [f1,d1] = vl_sift(I1) ;
        
%im2
  deptharray_obj = dep2;
        xyz2=get_xyzasus(deptharray_obj(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        %ATENCAO APAZ RGB VIRTUal e nao o DA IMAGEM BOA
        %         PILA
        rgbd2=get_rgbd(xyz2,im2,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
        
         I2 = single(rgb2gray(im2));
       [f2,d2] =  vl_sift(I2);
         [matches, scores] = vl_ubcmatch(d1,d2);
         
         %obtem posições xy e arredonda 
         pts1 = round(f1(1:2,matches(1,:)));
         %converte de xy para array/indice
         pt1 = pts1(1,:)+(pts1(2,:)-1)*640;
         %busca pontos 3D importantes de todos os pontos 3D
         xyz_pts1 = xyz1(pt1,:);
         
         
         pts2 = round(f2(1:2,matches(2,:)));
         pt2 = pts2(1,:)+(pts2(2,:)-1)*640;
         xyz_pts2 = xyz2(pt2,:);
         high_score_inliers = 0;
         
         for i=1:50
            
             
   
 
         perm = randperm(size(matches,2),6) ;
         P1 = xyz_pts1(perm,:);
         P2 = xyz_pts2(perm,:);
         [d,xx,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);
         
         
          xyz2in1=xyz_pts2*tr.T+ones(length(xyz_pts2),1)*tr.c(1,:);
          
          pila = xyz2in1 - xyz_pts1;
          norms = sqrt(sum(pila'.^2,1))
          
         inliers_id = find( norms < 30) %PODE NAO ESTAR BEM, ATENÇÃO RAMIRAGOIS
          
          if(length(inliers_id) > high_score_inliers)
              
              high_score_inliers = length(inliers_id);
              best_inliers = inliers_id;
              
          end

         end
         
         perm = best_inliers;
          P1 = xyz_pts1(perm,:);
         P2 = xyz_pts2(perm,:);
         [d,xx,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);
         
        
        pc1=pointCloud(xyz1);
            
        xyz2in1=xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:);
 
        pc2=pointCloud(xyz2in1);

        FG_pts=pcmerge(pc1,pc2,0.001)
        pcshow(FG_pts);
        
        
    xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
    xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', Depth_cam.K,1,0);
    rgbd1 = get_rgbd(xyz1, im1, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
    rgbd2 = get_rgbd(xyz2, im2, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
    pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
    pc2=pointCloud(xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:),'Color',reshape(rgbd2,[480*640 3]));
    close all
    %figure(1);hold off;
    %showPointCloud(pc1)
    pcshow(pcmerge(pc1,pc2,0.001));
    drawnow;

        
        
        
        
        
        
        
%%
np=6;
figure(1);x1=zeros(np,1);y1=x1;x2=y1;y2=x1;
for i=1:np,
    figure(1);
    [xa ya]=ginput(1);text(xa,ya,int2str(i));
    xa=fix(xa);ya=fix(ya);
    x1(i)=xa;y1(i)=ya;
    aux1=xyz1(sub2ind([480 640],ya,xa),:);
    figure(3);hold  on; plot3(aux1(1),aux1(2),aux1(3),'or','MarkerSize',10);
    hold off;
    figure(2);
    [xa ya]=ginput(1);text(xa,ya,int2str(i));
    xa=fix(xa);ya=fix(ya);
    x2(i)=xa;y2(i)=ya;
    aux1=xyz2(sub2ind([480 640],fix(ya),fix(xa)),:);
    figure(4);hold  on; plot3(aux1(1),aux1(2),aux1(3),'or','MarkerSize',10);
    hold off;drawnow;
end
%%
figure(1);hold on; plot(x1,y1,'*r');hold off;
figure(2);hold on;plot(x2,y2,'*r');hold off;
ind1=sub2ind(size(dep2),y1,x1);
ind2=sub2ind(size(dep2),y2,x2);
%%
P1=xyz1(ind1,:);
P2=xyz2(ind2,:);
inds=find((P1(:,3).*P2(:,3))>0);
P1=P1(inds,:);P2=P2(inds,:);




   deptharray_obj = deptharray1.*moldura;
        xyz1=get_xyzasus(deptharray_obj(:)*1000,[480 640],(1:640*480)',Depth_cam.K,1,0);
        
        rgbd=get_rgbd(xyz1,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
        
         I = single(rgb2gray(rgbd));
        %I1 = single(rgb2gray(im1));
        
        %[fa,da] = vl_sift(I1);
        [ramiro(k).f,ramiro(k).d] = vl_sift(I);


[d,xx,tr]=procrustes(P1,P2,'scaling',false,'reflection',false);





xyz21=xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:);
pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
pc2=pointCloud(xyz21,'Color',reshape(rgbd2,[480*640 3]));
figure(1);clf; showPointCloud(pc1);
figure(2);clf; showPointCloud(pc2);
pause;



%%
