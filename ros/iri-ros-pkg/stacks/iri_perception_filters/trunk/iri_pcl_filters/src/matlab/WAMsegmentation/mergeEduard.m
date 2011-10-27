%% 6.- overlapping, entwining and merging ;-)

close all
tol = 1e-2;

g = fspecial('gauss', [11 11], sqrt(11));

% selk = MoFA.mix > 0.01;
% k = sum(selk);
% colors_lab = MoFA.M(1:3,selk)';
% Qred = Q(selk,:);

k = klusters;
colors_lab = MoFA.M(1:3,:)';
Qred = Q;

for idx_k = 1:k
    imgQk = reshape(Q(idx_k,:)',640,480,1);
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
        color_diff = 0.95;
    elseif it == 2
        threshold_overlap = 0.25;
        color_diff = 0.5;
    elseif it == 3
        threshold_overlap = -1;
        color_diff = 0.95;
    elseif it == 4
        threshold_overlap = 0.25;
        color_diff = 0.5;
    elseif it == 5
        threshold_overlap = -1;
        color_diff = 0.95;
    else
        threshold_overlap = -1;
        color_diff = color_diff - 0.005;
    end
    
    mergeflag = false;
    for idx_k = 1:k
        for idx_l = idx_k+1:k
            if color_dist(idx_k,idx_l) < color_diff, continue; end;

            imgkl = max(imgk(:,:,[idx_k,idx_l]),[],3);
            %imgkl = sum(imgk(:,:,[idx_k,idx_l]),3);
            bimgkl = imgkl > tol;

            boverlapkl = and(bimgk(:,:,idx_k),bimgk(:,:,idx_l));
            overlapkl = imgkl .* boverlapkl;
            
            volk = sum(sum(imgk(:,:,idx_k)));
            voll = sum(sum(imgk(:,:,idx_l)));
            volklint = sum(sum(overlapkl));
            volkl = sum(sum(imgkl));
            
            bvolk = sum(sum(bimgk(:,:,idx_k)));
            bvoll = sum(sum(bimgk(:,:,idx_l)));
            bvolklint = sum(sum(boverlapkl));
            bvolkl = sum(sum(bimgkl));
            
            merge_factork = volklint./(volk + voll);% .* color_dist(idx_k,idx_l)
            merge_factorl = volklint./(volk + voll);% .* color_dist(idx_k,idx_l)
            
            merge_bfactork = bvolklint./bvolk;% .* color_dist(idx_k,idx_l)
            merge_bfactorl = bvolklint./bvoll;% .* color_dist(idx_k,idx_l)
            
            if merge_factork > threshold_overlap || merge_factorl > threshold_overlap
                imgk(:,:,idx_k) = imgkl;
                imgk(:,:,idx_l) = [];
                bimgk = imgk > tol;
                new_color = (colors_lab(idx_k,:)*volk + colors_lab(idx_l,:)*voll)/(1*(volk+voll));
                colors_lab(idx_k,:) = new_color;
                colors_lab(idx_l,:) = [];
                
                %Qred(idx_k,:) = sum([Q(idx_k,:); Q(idx_l,:)],1);
                Qred(idx_k,:) = reshape(imgkl',1,480*640);
                Qred(idx_l,:) = [];
                
                color_dist = sqrt(sum(abs( repmat(permute(colors_lab, [1 3 2]), [1 k-1 1]) ...
                    - repmat(permute(colors_lab, [3 1 2]), [k-1 1 1]) ).^2, 3));
                color_dist = -color_dist / max(max(color_dist)) +1;
                
                mergeflag = true;
                break;
            end

        end
        if mergeflag == true, 
            k = k - 1;
            break; 
        end;
    end
    
    if mergeflag == false && it < maxit+1
        mergeflag = true;
        it=it+1;
        
        [E,I] = max(Qred,[],1);
        cform = makecform('lab2srgb');
        colors_rgb = applycform(colors_lab(:,1:3),cform);
        image2 = colors_rgb(I',:);

        img_RGB = reshape(image2,640,480,3);

        img_RGB2(:,:,1) = img_RGB(:,:,1)';
        img_RGB2(:,:,2) = img_RGB(:,:,2)';
        img_RGB2(:,:,3) = img_RGB(:,:,3)';

        figure;
        imshow(img_RGB2)
    end
end

[E,I] = max(Qred,[],1);
cform = makecform('lab2srgb');
colors_rgb = applycform(colors_lab(:,1:3),cform);
image2 = colors_rgb(I',:);

img_RGB = reshape(image2,640,480,3);

img_RGB2(:,:,1) = img_RGB(:,:,1)';
img_RGB2(:,:,2) = img_RGB(:,:,2)';
img_RGB2(:,:,3) = img_RGB(:,:,3)';


figure;
imshow(he)

figure;
imshow(img_RGB2)

if cluster3d, imname = ['./output/merged'  num2str(id) 'nclust' num2str(klusters) 'inf3d']; 
else imname = ['./output/merged'  num2str(id) 'nclust' num2str(klusters)]; end;
saveas(gcf,imname,'jpg') 




for idx_k= 1:k
    figure
    imshow(imgk(:,:,idx_k))
    
    [u v] = find(imgk(:,:,idx_k) > tol);
    m_uv(idx_k,:) = mean([v,u]);
    c_uv(:,:,idx_k) = cov([v,u]);
    d_uv(idx_k) = det(c_uv(:,:,idx_k));
end

figure; hold on;
plot(m_uv(:,2),m_uv(:,1),'k.');
for idx_k= 1:k
plot_gaussian_ellipsoid([m_uv(idx_k,2),m_uv(idx_k,1)],diag([c_uv(2,2,idx_k),c_uv(1,1,idx_k)]))
end

seld = d_uv/max(d_uv) > 0.25;
imgk(:,:,seld) = [];
bimgk = imgk > tol;
colors_lab(seld,:) = [];
Qred(seld,:) = [];
k = k - sum(seld);

for idx_k= 1:k
    figure
    imshow(imgk(:,:,idx_k))
end
