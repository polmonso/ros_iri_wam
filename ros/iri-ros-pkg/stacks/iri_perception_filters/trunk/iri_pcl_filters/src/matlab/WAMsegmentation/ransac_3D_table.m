function [result_plane_coefficients, result_consensus]=ransac_3D_table(space3D)
%clear all; close all; load ransac_epnp; intwarning('off')
% Parameters ransac
n_points = size(space3D,1);
% n_ransac = 3; % Number of points to use in the minimal set
threshold = 3; % Max distance to be inlier
min_consensus = 1000;
max_iter_ransac = 1000; % 1000

%%%%%%%%%%%%%%

for iterator1=1:max_iter_ransac
    % Select n points randomly
    pts = randsample(n_points,3);
    
    points = space3D(pts,:);
    
    try
        P12 = points(2,:) - points(1,:);
        P13 = points(3,:) - points(1,:);
        plane_coefficients = zeros(4,1);
        plane_coefficients(1:3) = cross(P12,P13);
        
%         % Calculate plane
%         [u,s,v] = svd(points);
%         
%         plane_coefficients = zeros(4,1);
%         plane_coefficients(1:3) = v(:,end);
        
        plane_coefficients(4) = sum(plane_coefficients(1:3).*-points(1,:)'); % ax + by + cz + d = 0
        
        
        % Go from here -> calculate distances, and get number of consensus
        
        distances = zeros(n_points,1);
        denominador = 1 / sqrt( plane_coefficients(1)^2+plane_coefficients(2)^2+plane_coefficients(3)^2 );
        for iterator2=1:n_points
            % Distances to the plane
            actual_point = space3D(iterator2,:); % X,Y,Z
            distances(iterator2) = abs( plane_coefficients(1)*actual_point(1) + ...
                                        plane_coefficients(2)*actual_point(2) + ...
                                        plane_coefficients(3)*actual_point(3) + ...
                                        plane_coefficients(4)) * denominador;
        end % sum(plane_coefficients(1:3).*point1)+plane_coefficients(4) if it is == 0 then it belongs to the plane
        
        % Find consensus
        consensus = find(distances<threshold);
        num_consensus = size(consensus,1);
        
        if (iterator1 == 1)||(num_consensus > result_num_consensus)
            result_consensus = consensus;
            result_num_consensus = num_consensus;
            result_plane_coefficients = plane_coefficients;
        end
        
    catch exception
        num_consensus= 0;
        fprintf('RANSAC iteration failed: %d\n', exception.identifier);
    end

%     % Plot the plane
%     ti = -500:10:500; 
%     [XI,YI] = meshgrid(ti,ti);
%     ZI = griddata(space3D(pts,1),space3D(pts,2),-space3D(pts,3),XI,YI);
% 
%     
%     hold off; plot3(space3D(:,1), space3D(:,2), -space3D(:,3), 'k.');
%     hold on; grid; mesh(XI,YI,ZI); view(-90,0); plot3(space3D(pts,1), space3D(pts,2), -space3D(pts,3), 'r.'); %pause;
    
    if (num_consensus > min_consensus)
%         break;
    end
    
    if mod(iterator1,100)==0;
        fprintf('iter %d,max_consensus %d\n',iterator1,result_num_consensus);
    end
end
