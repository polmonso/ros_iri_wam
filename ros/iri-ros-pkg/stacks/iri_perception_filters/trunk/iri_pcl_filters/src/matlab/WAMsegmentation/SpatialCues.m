function [hC vC] = SpatialCues(im,sel)
g = fspecial('gauss', [11 11], sqrt(11));
% g = fspecial('gauss', [51 51], sqrt(51));
if sel, 
    dy = fspecial('log');
    vf = conv2(g, dy, 'valid');
else
%     dy = fspecial('sobel');
%     dxy = [-2 -1 0; -1 0 1; 0 1 2].*1;
%     dyx = [0 1 2; -1 0 1; -2 -1 0].*1;
    dy  = [ 1  2  3  2  1; 
            2  3  5  3  2; 
            0  0  0  0  0; 
           -2 -3 -5 -3 -2; 
           -1 -2 -3 -2 -1]./2;
    dyx = [ 0  2  3  2  1; 
           -2  0  3  5  2; 
           -3 -3  0  3  3; 
           -2 -5 -3  0  2; 
           -1 -2 -3 -2  0]./2;
    dxy = [-1 -2 -3 -2  0; 
           -2 -5 -3  0  2; 
           -3 -3  0  3  3; 
           -2  0  3  5  2; 
            0  2  3  2  1]./2;
    vdy = conv2(g, dy, 'valid');
    vdxy = conv2(g, dxy, 'valid');
    vdyx = conv2(g, dyx, 'valid');
%     vf = vdy + vdxy;
end
sz = size(im);

vC = zeros(sz(1:2));
hC = vC;

for b=1:size(im,3)
    vC = max(vC, abs(imfilter(im(:,:,b), vdy, 'symmetric')));
    vC = max(vC, abs(imfilter(im(:,:,b), vdyx, 'symmetric')));
    hC = max(hC, abs(imfilter(im(:,:,b), vdy', 'symmetric')));
    hC = max(hC, abs(imfilter(im(:,:,b), vdxy, 'symmetric')));
end
% for b=1:size(im,3)
%     vC = max(vC, abs(imfilter(im(:,:,b), vf, 'symmetric')));
%     hC = max(hC, abs(imfilter(im(:,:,b), vf', 'symmetric')));
% end


