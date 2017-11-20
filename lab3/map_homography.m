parede1=imread('parede1.jpg');
parede2=imread('parede2.jpg');
%lab2 feito pelo nuno
imshow(parede1);
[u1 v1]=ginput(4);
figure;
imshow(parede2);
[u2 v2]=ginput(4);

pts=[0 0; 200 0; 200 300; 0 300];
t1=cp2tform([u1 v1],pts,'projective');
t2=cp2tform([u2 v2],pts,'projective');

im1=imtransform(parede1,t1,'Xdata',[-400 800],'YData',[-250 600]);
im2=imtransform(parede2,t2,'Xdata',[-400 800],'YData',[-250 600]);

close all hidden;

figure;
imshow(im1);

figure;
imshow(im2);

figure;
imagesc(uint8(double(im1)*.5+double(im2)*.5))