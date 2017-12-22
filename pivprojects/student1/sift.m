myDir = '../maizena/rgb_image';
ext_img = '.png.';
im1 = imread([myDir '1_' int2str(9) ext_img]);
im2 = imread([myDir '2_' int2str(9) ext_img]);

I1 = single(rgb2gray(im1));
I2 = single(rgb2gray(im2));

[fa,da] = vl_sift(I1);
[fb,db] = vl_sift(I2);

[matches,scores] = vl_ubcmatch(da,db);

subplot (1,2,1);
imshow (uint8(I1));
hold on;
plot (fa(1,matches(1,:)), fa(2, matches(1,:)), 'b*');

subplot (1,2,2);
imshow (uint8 (I2));
hold on;
plot (fb(1, matches(2,:)), fb(2, matches (2,:)), 'r*');


figure(2) ; clf ;
imagesc(cat(2, I1, I2)) ;

xa = fa(1,matches(1,:)) ;
xb = fb(1,matches(2,:)) + size(I1,2) ;
ya = fa(2,matches(1,:)) ;
yb = fb(2,matches(2,:)) ;

hold on ;
h = line([xa ; xb], [ya ; yb]) ;
set(h,'linewidth', 1, 'color', 'b') ;

vl_plotframe(fa(:,matches(1,:))) ;
fb(1,:) = fb(1,:) + size(I1,2) ;
vl_plotframe(fb(:,matches(2,:))) ;
axis image off ;