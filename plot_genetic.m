k = readmatrix("genetic_algorithm_out.txt");
hold on;
for j = 3:size(k,1)
    curp = k(j,:);
    plot(j*ones(1,size(k,2))-2,curp/54,'m.--');
end
mo = mean(k,2)/54;
plot((3:size(k,1))-2,mo(3:size(mo,1),:),'k-');