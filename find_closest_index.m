function closest_index = find_closest_index(x, y, points)

    distances = sqrt((points(:,1) - x).^2 + (points(:,2) - y).^2);
    
    % Find the index of the minimum distance
    [~, closest_index] = min(distances);

    closest_index = mod(closest_index, size(points, 1))+1;
end