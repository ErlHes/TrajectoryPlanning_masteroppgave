function clean = removeNaN(matrix);

[rows, ~] = size(matrix);
clean = [];

for i = 1:rows
    temp = rmmissing(matrix(i,:));
    clean = [clean;temp];
end
