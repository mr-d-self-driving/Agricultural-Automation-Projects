global bitmap occuGrid bitpos trees trees_percieved


d_errors = [];
r_errors = [];

for i = 1 : length(trees_percieved)
    dist = [];    
    for j = 1 : length(trees)
        dist = [dist; sqrt((trees_percieved(i,1)-trees(j,1))^2 + (trees_percieved(i,2)-trees(j,2))^2)];
    end
    
    [error, jsave] = min(dist);
    d_errors = [d_errors; error];
    r_errors = [r_errors; (trees_percieved(i,3)-trees(jsave,3))];
end

r_errors = nonzeros(r_errors); 
d_errors = nonzeros(d_errors); 

figure(1)
histogram(r_errors)
title('Radius Error')

figure(2)
histogram(d_errors)
title('Position(dist.) Error')