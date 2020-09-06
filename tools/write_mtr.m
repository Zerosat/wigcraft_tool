function out = write_mtr(filename,sizes)
		
	poly_file_handle = fopen(filename,'w');
	
	fprintf(poly_file_handle,'# Sizing function definition\n');
	fprintf(poly_file_handle,'%d 1\n',size(sizes,1));
	
	for i = 1:1:size(sizes,1)
		fprintf(poly_file_handle,'%d\n',sizes(i,:));
	end
	
	out = true;
	fclose(poly_file_handle);
end