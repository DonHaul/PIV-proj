imgmed=zeros(480,640,45);
for i=1:45,
    load(['depth1_' int2str(i) '.mat']);
    imagesc(depth_array);
    imgmed(:,:,i)=double(depth_array)/1000;
    pause(.1)
    drawnow;
end
bg=median(imgmed,3);
d=dir('depth1*.mat');
for i=1:length(d),
    load(d(i).name);
    imagesc(abs(double(depth_array)/1000-bg)>.25);
    colormap(gray);
    pause(1);
end
    