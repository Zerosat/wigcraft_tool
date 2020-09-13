k = readmatrix("genetic_algorithm_out2.txt");
hold on;
for j = 1:size(k,1)
    curp = k(j,:);
    plot(j*ones(1,size(k,2)),curp,'m.--');
end
plot(1:size(k,1),mean(k,2),'k-');