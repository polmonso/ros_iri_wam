function number_of_objects = colorSegmentationWAM(filename, id)

close all; %clear all; 
% color clustering
% addpath(genpath('./input'));
addpath(genpath('./mfa'));

%% 0.- options
cluster3d = false;  % MoFA with 3D info
ransac_ds = 10;     % RANSAC downsampling factor
mofa_ds = ransac_ds*5; % MoFA downsampling factor
klusters = 25;      % Number of desired clusters
lab_space = true;   % Work with Lab / RGB color space

display('1.- load image');
display(id);

if(nargin < 2) 
    id = 3;
end
if (nargin < 1)
   filename = ['last_image.dat'];
else
%     filename = ['frameXYZRGB0'  num2str(id)  '.dat'];
    filename = [filename '.dat'];
    display(filename);
%     filename = ['input/last_image7.dat'];
end

A = importdata(filename);
sel_nan = isnan(prod(A(:,3:5),2));

nrows = size(unique(A(:,2)),1); % originally 640
row_dowsmaplig = 640/nrows;
ncolumns = size(unique(A(:,1)),1); % originally 480
col_dowsmaplig = 480/ncolumns;
total_input_downsampling = row_dowsmaplig * col_dowsmaplig;

image = reshape(A(:,6:8),nrows,ncolumns,3);
he(:,:,1) = image(:,:,1)';
he(:,:,2) = image(:,:,2)';
he(:,:,3) = image(:,:,3)';
he = cast(he,'uint8');
imshow(he);

imname = ['./output/image'  num2str(id)];
saveas(gcf,imname,'jpg');
save(imname,'A','-ascii');

display('2.1- find table plane');
ANoNaN = A(~sel_nan,:);
sel = 1:ransac_ds:size(ANoNaN,1);
sel_len = length(sel);
Y = ANoNaN(sel,:);

[plane_coefficients, consensus]=ransac_3D_table(Y(:,3:5));

%     figure; view(150,25); hold on;
% plot3(Y(:,3), -Y(:,4), -Y(:,5), 'k.');
% plot3(Y(consensus,3), -Y(consensus,4), -Y(consensus,5), 'r.');
ti = -1000:10:1000;
[XI,YI] = meshgrid(ti,ti);
ZI = griddata(Y(consensus,3),-Y(consensus,4),-Y(consensus,5),XI,YI);
%     hold on; grid; mesh(XI,YI,ZI);

display('2.2.- Eliminate the table and everything beneath it');

tolerance = 10;
projectedZ = (ANoNaN(:,3:4)*plane_coefficients(1:2)+plane_coefficients(4))/-plane_coefficients(3);

height_difference = ANoNaN(:,5) - (projectedZ - tolerance);

indexes = find(height_difference>0);

%     figure; view(150,25); hold on;
%     plot3(ANoNaN(:,3), -ANoNaN(:,4), -ANoNaN(:,5), 'k.');
%     plot3(ANoNaN(indexes,3), -ANoNaN(indexes,4), -ANoNaN(indexes,5), 'r.');

% label non-cloth points
ANoNaN(indexes,6:8) = -255;
ANoNaN(indexes,3:5) = NaN;
A(~sel_nan,:) = ANoNaN;
A(sel_nan,6:8) = -255;
sel_nan = isnan(prod(A(:,3:5),2));

display('3.- convert RGB to Lab')

cform = makecform('srgb2lab');
Argb = A(:,6:8);
if lab_space
    Alab = applycform(Argb./255,cform);
    Blab(:,1:3) = Alab; Blab(:,4:5) = A(:,1:2);
    if cluster3d, Blab(:,6:8) = A(:,3:5); end;
else
    Brgb(:,1:3) = Argb; Brgb(:,4:5) = A(:,1:2);
    if cluster3d, Brgb(:,6:8) = A(:,3:5); end;
end

display('4.1- cluster in color + uv space');

if lab_space
    if cluster3d, BlabNoNaN = Blab(~sel_nan,:);
    else BlabNoNaN = Blab; end
    sel = 1:mofa_ds:size(BlabNoNaN,1);
    X = BlabNoNaN(sel,:)';
else
    if cluster3d, BrgbNoNaN = Brgb(~sel_nan,:);
    else BrgbNoNaN = Brgb; end
    sel = 1:mofa_ds:size(BrgbNoNaN,1);
    X = BrgbNoNaN(sel,:)';
end

d = 1; % Why 1 ?????????????????????????
k = klusters;
iso = 0;
eq = 0;
dia = 1;
[LogL,MoFA,Q] = mfa(X,d,k,iso,eq,dia);

display('4.2.- Reduce number of clusters if their mix is insufficient')
selk = MoFA.mix > 0.005;
selk_index = find(selk==1);

aux_MoFA.M = MoFA.M(:,selk_index);
aux_MoFA.W = MoFA.W(:,selk_index);
aux_MoFA.Psi = MoFA.Psi(:,selk_index);
aux_MoFA.mix = MoFA.mix(selk_index,:);

MoFA = aux_MoFA;

aux_Q = Q(selk,:);
Q = aux_Q;

k = size(MoFA.mix,1);
klusters = size(MoFA.mix,1);

display('4.3.- show clusters')

% Color space
%     figure; hold on;
%     plot3(X(1,:)',X(2,:)',X(3,:)','r.')
%     for idx=1:size(MoFA.M(1:3,:),2)
%         plot_gaussian_ellipsoid(MoFA.M(1:3,idx),diag(MoFA.Psi(1:3,idx)));
%     end
%
%     % UV space
%     figure; hold on;
%     plot(X(4,:)',X(5,:)','r.')
%     for idx=1:size(MoFA.M(4:5,:),2)
%         plot_gaussian_ellipsoid(MoFA.M(4:5,idx),diag(MoFA.Psi(4:5,idx)));
%     end
%
%     % XYZ space
%     if cluster3d
%         figure; hold on;
%         plot3(X(6,:)',X(7,:)',X(8,:)','r.')
%         for idx=1:size(MoFA.M(6:8,:),2)
%             plot_gaussian_ellipsoid(MoFA.M(6:8,idx),diag(MoFA.Psi(6:8,idx)));
%         end
%     end

% % UV + 1D color
% figure; hold on;
% cform = makecform('lab2srgb');
% Xrgb = applycform(X(1:3,:)',cform);
% scatter3(X(4,:)',X(5,:)',mean(X(1:3,:))',ones(length(X),1)*18,Xrgb,'filled')
% for idx=1:size(MoFA.M(4:5,:),2)
%     plot_gaussian_ellipsoid([MoFA.M(4:5,idx); mean(MoFA.M(1:3,idx))],diag([MoFA.Psi(4:5,idx);mean(MoFA.Psi(1:3,idx))]));
% end

display('5.- segment image')

% image reconstruction
Qexp = zeros(klusters,nrows*ncolumns);
Qori = Q;
if mofa_ds == 1
    [E,I] = max(Q,[],1);
else
    if cluster3d
        Xcomp = [X(4:5,:)' X(1:3,:)' X(6:8,:)'];
        if lab_space, Acomp = [A(:,1:2) Alab A(:,6:8)];
        else Acomp = [A(:,1:2) Argb A(:,6:8)]; end;
    else
        Xcomp = [X(4:5,:)' X(1:3,:)'];
        if lab_space, Acomp = [A(:,1:2) Alab];
        else Acomp = [A(:,1:2) Argb]; end;
    end
    [Ex,Ix] = max(Q,[],1);
    Xcomp_len = length(Xcomp);
    Acomp_len = length(Acomp);
    I = zeros(1,Acomp_len);
    uv_dist = zeros(1,Acomp_len);
    
    background_counter = 0;
    
    for pix = 1:Acomp_len
        % check cluster
        if ( (sel_nan(pix)) && (cluster3d) ) % only in case we are using 3D in MoFA
            y = Acomp(pix,1:5);
            pix_dist = sqrt(sum(abs(Xcomp(:,1:5) - y(ones(Xcomp_len,1),:)).^2, 2));
        else
            y = Acomp(pix,:);
            pix_dist = sqrt(sum(abs(Xcomp(:,1:5) - y(ones(Xcomp_len,1),:)).^2, 2));
        end
        [d,idx] = min(pix_dist,[],1);
        I(pix) = Ix(idx);
        Qexp(:,pix) = Q(:,idx);
        
        if (sel_nan(pix))
            background_counter = background_counter + 1;
            background_cluster_vector(background_counter) = I(pix);
        end
    end
end

background_cluster = mode(background_cluster_vector); % The background cluster is the one containing the most NaN's


% colors_rgb = varycolor(C);
if lab_space
    colors_lab = MoFA.M(1:3,:)';
    cform = makecform('lab2srgb');
    colors_rgb = applycform(colors_lab,cform);
else
    colors_rgb = MoFA.M(1:3,:)'./255;
end
image2 = colors_rgb(I',:);

img_RGB = reshape(image2,nrows,ncolumns,3);

img_RGB2(:,:,1) = img_RGB(:,:,1)';
img_RGB2(:,:,2) = img_RGB(:,:,2)';
img_RGB2(:,:,3) = img_RGB(:,:,3)';

figure;
imshow(img_RGB2)

if cluster3d, imname = ['./output/image'  num2str(id) 'nclust' num2str(klusters) 'inf3d'];
else imname = ['./output/image'  num2str(id) 'nclust' num2str(klusters)]; end;
saveas(gcf,imname,'jpg');

% % mean shift
% [fimage, labels, modes, regSize, grad, conf] = edison_wrapper(image,@RGB2Luv);
% fimage2(:,:,1) = fimage(:,:,1)';
% fimage2(:,:,2) = fimage(:,:,2)';
% fimage2(:,:,3) = fimage(:,:,3)';
% cform = makecform('lab2srgb');
% fimrgb = applycform(double(fimage2),cform);
% figure
% imshow(fimrgb)

display('6.- overlapping, entwining and merging ;-)');

close all
tol = 1e-2;
Q = Qexp;
g = fspecial('gauss', [11 11], sqrt(11));

% selk = MoFA.mix > 0.01;
% k = sum(selk);
% colors_lab = MoFA.M(1:3,selk)';
% Qred = Q(selk,:);

k = klusters;
colors_lab = MoFA.M(1:3,:)';
Qred = Q;

% Smoothing the partial belonging to a class
for idx_k = 1:k
    imgQk = reshape(Q(idx_k,:)',nrows,ncolumns,1);
    imgk(:,:,idx_k) = imgQk';
    
    imgk(:,:,idx_k) = imfilter(imgk(:,:,idx_k), g, 'symmetric');
    
    %     figure;% hold  on;
    %     imshow(imgk(:,:,idx_k));
end

bimgk = imgk > tol;

color_dist = sqrt(sum(abs( repmat(permute(colors_lab, [1 3 2]), [1 k 1]) ...
    - repmat(permute(colors_lab, [3 1 2]), [k 1 1]) ).^2, 3));
color_dist = -color_dist / max(max(color_dist)) +1;
% for idx_k = 1:k
%     figure;
%     imshow(imgk(:,:,idx_k));
%     strtit = sprintf('color distance = %02f',color_dist(idx_k,1));
%     set(gcf,'name',strtit)
% end
% color_dist
% pause

mergeflag = true;
maxit = 10;
it = 1;
while(mergeflag)
    if it == 1
        threshold_overlap = -1;
        color_diff = 0.88;
    elseif it == 2
        threshold_overlap = 0.15;
        color_diff = 0.75;
    elseif it == 3
        threshold_overlap = -1;
        color_diff = 0.88;
    elseif it == 4
        threshold_overlap = 0.25;
        color_diff = 0.5;
    elseif it == 5
        threshold_overlap = -1;
        color_diff = 0.88;
    else
        threshold_overlap = -1;
        color_diff = color_diff - 0.005;
    end
    
    mergeflag = false;
    for idx_k = 1:k
        for idx_l = idx_k+1:k
            if ( color_dist(idx_k,idx_l) < color_diff ) %|| (idx_k == background_cluster) || (idx_l == background_cluster)
                continue;
            end;
            
            imgkl = max(imgk(:,:,[idx_k,idx_l]),[],3);
            %imgkl = sum(imgk(:,:,[idx_k,idx_l]),3);
            bimgkl = imgkl > tol;
            
            boverlapkl = and(bimgk(:,:,idx_k),bimgk(:,:,idx_l));
            overlapkl = imgkl .* boverlapkl;
            
            volk = sum(sum(imgk(:,:,idx_k)));
            voll = sum(sum(imgk(:,:,idx_l)));
            volklint = sum(sum(overlapkl));
            volkl = sum(sum(imgkl));
            
            %                 bvolk = sum(sum(bimgk(:,:,idx_k)));
            %                 bvoll = sum(sum(bimgk(:,:,idx_l)));
            %                 bvolklint = sum(sum(boverlapkl));
            %                 bvolkl = sum(sum(bimgkl));
            
            merge_factork = volklint./(volk + voll);% .* color_dist(idx_k,idx_l)
            merge_factorl = volklint./(volk + voll);% .* color_dist(idx_k,idx_l)
            
            %                 merge_bfactork = bvolklint./bvolk;% .* color_dist(idx_k,idx_l)
            %                 merge_bfactorl = bvolklint./bvoll;% .* color_dist(idx_k,idx_l)
            
            if (merge_factork > threshold_overlap) || (merge_factorl > threshold_overlap)
                if (background_cluster > idx_l)
                    background_cluster = background_cluster - 1;
                end
                
                imgk(:,:,idx_k) = imgkl;
                imgk(:,:,idx_l) = [];
                bimgk = imgk > tol;
                new_color = (colors_lab(idx_k,:)*volk + colors_lab(idx_l,:)*voll)/(1*(volk+voll));
                colors_lab(idx_k,:) = new_color;
                colors_lab(idx_l,:) = [];
                
                %Qred(idx_k,:) = sum([Q(idx_k,:); Q(idx_l,:)],1);
                Qred(idx_k,:) = reshape(imgkl',1,ncolumns*nrows);
                Qred(idx_l,:) = [];
                
                color_dist = sqrt(sum(abs( repmat(permute(colors_lab, [1 3 2]), [1 k-1 1]) ...
                    - repmat(permute(colors_lab, [3 1 2]), [k-1 1 1]) ).^2, 3));
                color_dist = -color_dist / max(max(color_dist)) +1;
                
                mergeflag = true;
                break;
            end
            
        end
        if (mergeflag == true)
            k = k - 1;
            break;
        end
    end
    
    if ( (mergeflag == false) && (it < maxit+1) )
        mergeflag = true;
        it=it+1;
        
        [E,I] = max(Qred,[],1);
        cform = makecform('lab2srgb');
        colors_rgb = applycform(colors_lab(:,1:3),cform);
        image2 = colors_rgb(I',:);
        
        img_RGB = reshape(image2,nrows,ncolumns,3);
        
        img_RGB2(:,:,1) = img_RGB(:,:,1)';
        img_RGB2(:,:,2) = img_RGB(:,:,2)';
        img_RGB2(:,:,3) = img_RGB(:,:,3)';
        
        %             figure;
        %             imshow(img_RGB2)
    end
end

[E,I] = max(Qred,[],1);
cform = makecform('lab2srgb');
colors_rgb = applycform(colors_lab(:,1:3),cform);
image2 = colors_rgb(I',:); % image 2 contiene los labels con los pixeles sin formato de imagen

number_of_objects = size(Qred,1)-1;

display('7.- Eliminate Noisy custers');

minimum_number_to_consider = round( 2500 / total_input_downsampling );
for iterator=1:number_of_objects+1
    indexes = find(I==iterator);
    number_of_points_in_cluster =  size(indexes,2);
    disp(number_of_points_in_cluster);
    
    if (number_of_points_in_cluster < minimum_number_to_consider)
        I(indexes) = background_cluster;
        image2(indexes,:) = zeros(number_of_points_in_cluster,3);
        number_of_objects = number_of_objects - 1;
    end
end

display('8.- Save output file');

B =  round([I' image2*255]); % image in 0-255 scale

if (nargin == 1)
    output_filename = [filename '_labelled.dat'];
else
    output_filename = ['last_image_labelled.dat'];
end

dlmwrite(output_filename, B, 'delimiter', ' ', 'precision', '%d');

display('9.- Reshape image to show pretty printing');

img_RGB = reshape(image2,nrows,ncolumns,3);

img_RGB2(:,:,1) = img_RGB(:,:,1)';
img_RGB2(:,:,2) = img_RGB(:,:,2)';
img_RGB2(:,:,3) = img_RGB(:,:,3)';


figure;
imshow(he);

figure;
imshow(img_RGB2);

if cluster3d, imname = ['./output/image'  num2str(id) 'nclust' num2str(klusters) 'merged3d'];
else imname = ['./output/image'  num2str(id) 'nclust' num2str(klusters) 'merged'];  end;
saveas(gcf,imname,'jpg');


