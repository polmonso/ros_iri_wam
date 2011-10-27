% bimgk = aux_bimgk;
bimgk = imgk;
k = aux_k;

clusters = 1:k;

change = true;
while change
    change = false;
    
    for idx_k = 1:k
        for idx_l = idx_k+1:k
            figure(1);
            imshow(bimgk(:,:,idx_k));
            number_of_points_in_k = size(find(bimgk(:,:,idx_k) > 0),1);
            figure(2);
            imshow(bimgk(:,:,idx_l));
            number_of_points_in_l = size(find(bimgk(:,:,idx_l) > 0),1);
            
            merge_matrix = (bimgk(:,:,idx_k)+bimgk(:,:,idx_l))/2;
            
            figure(3);
            imshow(merge_matrix);

            bin_matrix1 = bimgk(:,:,idx_k) > 0.05;
            bin_matrix2 = bimgk(:,:,idx_l) > 0.05;
            bin_merge_matrix = and(bin_matrix1(:,:,idx_k),bin_matrix2(:,:,idx_l));
            
            number_of_points_in_merge = size(find(bin_merge_matrix > 0),1);
            
            if ((number_of_points_in_merge > number_of_points_in_k*0.3) || (number_of_points_in_merge > number_of_points_in_k*0.3))
                bimgk(:,:,idx_k) = merge_matrix;
                if (idx_l == k)
                    bimgk2 = bimgk(:,:,[1:idx_l-1]);
                else
                    bimgk2 = bimgk(:,:,[1:idx_l-1 idx_l+1:k]);
                end
                bimgk = bimgk2;
                clusters(idx_l) = clusters(idx_k);
                k = k-1;
                
                change = true;
                
                figure(4);
                imshow(bimgk(:,:,idx_k)); pause;
            end
            
            if change
                break;
            end
        end
        if change
            break;
        end
    end
end


