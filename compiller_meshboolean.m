clearvars

path_to_libigl = 'E:\Pablo\Documentos\Github\Ekranoplan-model\bin\dependencies\libigl';
path_to_eigen = ['E:\Pablo\Documentos\Github\Ekranoplan-model\bin\dependencies\eigen'];
path_to_cgal = 'E:\Pablo\Documentos\Github\Ekranoplan-model\bin\dependencies\cgal\include';
path_to_boost = 'E:\Pablo\Documentos\Github\Ekranoplan-model\bin\dependencies\boost';


MEXOPTS = {'-v','-largeArrayDims','-DMEX'};

EIGEN_INC = ['-I' path_to_eigen];
CGAL = ['-I' path_to_cgal];
BOOST = ['-I' path_to_boost];
LIBIGL = ['-I' path_to_libigl];

mex(MEXOPTS{:},LIBIGL,EIGEN_INC,CGAL,BOOST,'mesh_boolean.cpp');

