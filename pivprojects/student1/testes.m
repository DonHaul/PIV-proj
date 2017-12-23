clear


base_data_dir='C:\Users\ASUS\Desktop\AQUI\PIV\PIV-proj\room\';
%base_data_dir='C:\Users\ASUS\Desktop\corredor\corredor1\';

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

corre='[objects,cam1toW,cam2toW] = track3D_part( im1, im2,cam_params);';



h=tic;
eval(corre);
tt=toc(h)
%%
caminho={};

out{1}=objects;
out{2}=cam1toW;
out{3}=cam2toW;
caminho=out;
    obj=caminho{1};
    objsinframe=zeros(length(obj),length(im1));
    for j=1:length(obj),
        for k=1:length(obj(j).frames_tracked),
            objsinframe(j,obj(j).frames_tracked(k))=1;
        end
    end
    R1=(caminho{1,2}.R);
    T1=(caminho{1,2}.T');
    R2=(caminho{1,3}.R);
    T2=(caminho{1,3}.T');
    for j=1:length(im1),
        load(im1(j).depth);
        xyz1=get_xyzasus(depth_array(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
        load(im2(j).depth);
        xyz2=get_xyzasus(depth_array(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
        xyz=[(R1*xyz1'+repmat(T1,[1,640*480]))';(R2*xyz2'+repmat(T2,[1,640*480]))'];
        pc=pointCloud([xyz(:,1) xyz(:,3) -xyz(:,2)]);%MATLAB AXIS!
        figure(1);
        showPointCloud(pc);
        view([.2 -.2 .05]);
        hold on;
        indsob=find(objsinframe(:,j));
        for k=1:length(indsob),
            ind=find(obj(indsob(k)).frames_tracked==j);
            combs=combnk((1:8),2)';
            xs=obj(indsob(k)).X(ind,:);
            ys=obj(indsob(k)).Z(ind,:);
            zs=-obj(indsob(k)).Y(ind,:);
            line([xs(combs(1,:));xs(combs(2,:))],[ys(combs(1,:));ys(combs(2,:))],[zs(combs(1,:));zs(combs(2,:))],'LineWidth',2);
        end
        hold off;
        pause;
    end
