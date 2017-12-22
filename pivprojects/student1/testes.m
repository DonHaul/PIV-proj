clear


base_data_dir='C:\Users\ASUS\Desktop\AQUI\PIV\PIV-proj\maizena\';
d1=dir([base_data_dir 'depth1*']);
d2=dir([base_data_dir 'depth2*']);
r1=dir([base_data_dir 'rgb_image1_*']);
r2=dir([base_data_dir 'rgb_image2_*']);
for i=1:length(d1),
    im1(i).rgb=[base_data_dir r1(i).name];
    im2(i).rgb=[base_data_dir r2(i).name];
    im1(i).depth=[base_data_dir d1(i).name];
    im2(i).depth=[base_data_dir d2(i).name];
end
%load calibration data
load cameraparametersAsus;
base.R = eye(3);
base.T = zeros(1,3);
camera2.R = [0.8048 ,-0.1051,    0.5842;   0.0189,    0.9882 ,   0.1517;    -0.5932  , -0.1110  ,  0.7973 ];
camera2.T = [0.7477 ,-0.1238, 0.1679];

corre='[objects] = track3D_part1( im1, im2,cam_params, base,camera2 );';



h=tic;
eval(corre);
tt=toc(h);