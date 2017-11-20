%%
%hi

IM = imread('rgb_image_10.png');
imshow(IM);
load depth_10
figure
imagesc(depth_array);
load CalibrationData
matrix = Depth_cam.K;
P= zeros (640*480,3);
Zd = double(depth_array)/1000;



%%
%minha parte
K_inv = inv(matrix);
auxi = 0;
for u =1:640
    for v=1:480
        auxi = auxi+1;
        aux = K_inv * Zd(v,u) * [u;v;1];
        P(auxi,:) = aux';
    end
end
%%
%hello

NOVO = pointCloud(P);
figure
showPointCloud(NOVO);

