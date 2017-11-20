%u = k_x*(X/Z) + c_x
%v = k_y*(X/Z) + c_y



% MATRIX LINEAR TRANSFORMATION
% IMAGE FUNCTION :^((;))




IM = imread('rgb_image_10.png');
load depth_10
u = ones(480,1)*(1:640);
v = (1:480)'*ones (1,640);
Zd = double(depth_array)/1000;
load CalibrationData

P = inv(Depth_cam.K)*[reshape(u.*Zd,[1 640*480]);reshape(v.*Zd,[1 640*480]);reshape(Zd,[1 640*480])];

%%
chimz = RGB_cam.K*[R_d_to_rgb T_d_to_rgb]*[P;ones(1,(640*480))];

u2 = chimz(1,:)./chimz(3,:);
v2 = chimz(2,:)./chimz(3,:);


c= zeros(3,640*480);
for i=1:480*640
    c(:,i) = [IM(v2(i),u2(i),1);IM(v2(i),u2(i),2);IM(v2(i),u2(i),3)]
end


pc = pointCloud(P','color',c');




