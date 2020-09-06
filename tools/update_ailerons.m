% This code updates the ailerons with a new angle and position. 
ABUval = mean(ABUVo,1);
ABUval = [ABUval(1) ABUval(2) max(ABUVo(:,3))];
ABUvct = [1 0 0];
ABUVi = (rodrigues_rot(ABUVo - ABUval,ABUvct,deg2rad(angle_up)) + ABUval)*rotx(deg2rad(curpitch))*roty(deg2rad(curyaw))*rotz(deg2rad(curroll));
ABRval = mean([-0.1883 -0.10615 -1.421;-0.34995 0.22635 -1.8125]);
ABRvct = [-0.1883 -0.10615 -1.421] - [-0.34995 0.22635 -1.8125];
ABRVi = (rodrigues_rot(ABRVo - ABRval,ABRvct,deg2rad(angle_back)) + ABRval)*rotx(deg2rad(curpitch))*roty(deg2rad(curyaw))*rotz(deg2rad(curroll));
ABLval = mean([0.1883 -0.10615 -1.421;0.34995 0.22635 -1.8125]);
ABLvct = [0.1883 -0.10615 -1.421] - [0.34995 0.22635 -1.8125];
ABLVi = (rodrigues_rot(ABLVo - ABLval,ABLvct,deg2rad(angle_back)) + ABLval)*rotx(deg2rad(curpitch))*roty(deg2rad(curyaw))*rotz(deg2rad(curroll));
AWRval = mean([-1.314 -0.1152 0.3634;-1.625 0.2472 0.22]);
AWRvct = [-1.314 -0.1152 0.3634] - [-1.625 0.2472 0.22];
AWRVi = (rodrigues_rot(AWRVo - AWRval,AWRvct,deg2rad(angle_wing_R)) + AWRval)*rotx(deg2rad(curpitch))*roty(deg2rad(curyaw))*rotz(deg2rad(curroll));
AWLval = mean([1.314 -0.1152 0.3634;1.625 0.2472 0.22]);
AWLvct = [1.314 -0.1152 0.3634] - [1.625 0.2472 0.22];
AWLVi = (rodrigues_rot(AWLVo - AWLval,AWLvct,deg2rad(angle_wing_L)) + AWLval)*rotx(deg2rad(curpitch))*roty(deg2rad(curyaw))*rotz(deg2rad(curroll));