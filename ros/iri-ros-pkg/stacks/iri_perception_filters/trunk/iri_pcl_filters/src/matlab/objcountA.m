load('last_image_labelled.dat')
if size(last_image_labelled)[1] == 307200
    rows = 640;
    cols = 480;
elseif size(last_image_labelled)[1] == 1310720
    rows = 1024;
    cols = 1280;
end

fprintf(1,'r: %d c: %d\n',rows,cols);
% A = last_image_labelled(1:cols/2,1);
% B = last_image_labelled(cols/2+1:cols,1);
% for i=2:rows
%     A = [A; last_image_labelled((i-1)*cols+1:(i-1)*cols+cols/2,1)];
%     B = [B; last_image_labelled((i-1)*cols+cols/2+1:(i-1)*cols+cols,1)];
% end

AB = reshape(last_image_labelled(:,1),640,480);
AB = AB';
image(AB);

A = AB(:,1:cols/2);
B = AB(:,cols/2+1:cols);
figure;
image(A);
figure;
image(B);