% This code initializes ailerons (position and scale)

% Apply the new position
[ABRVo, ABRFo] = deal(ail_backR.Points + VFo(100,:) - ail_backR.Points(24,:), ail_backR.ConnectivityList);
[ABLVo, ABLFo] = deal(ail_backL.Points + VFo(467,:) - ail_backL.Points(26,:), ail_backL.ConnectivityList);
[ABUVo, ABUFo] = deal(ail_backU.Points + VFo(26,:) - ail_backU.Points(9,:) + [-0.01 0 0], ail_backU.ConnectivityList);
[AWRVo, AWRFo] = deal(ail_wingR.Points + VFo(838,:) - ail_wingR.Points(44,:), ail_wingR.ConnectivityList);
[AWLVo, AWLFo] = deal(ail_wingL.Points + VFo(630,:) - ail_wingL.Points(41,:), ail_wingL.ConnectivityList);

% Correct bad orientations
ABRFo = unifyMeshNormals(ABRFo,ABRVo,'alignTo','out');
ABLFo = unifyMeshNormals(ABLFo,ABLVo,'alignTo','out');
ABUFo = unifyMeshNormals(ABUFo,ABUVo,'alignTo','out');
AWRFo = unifyMeshNormals(AWRFo,AWRVo,'alignTo','out');
AWLFo = unifyMeshNormals(AWLFo,AWLVo,'alignTo','out');

% The STL scale factor (for a proper merge)
incfactor = 1.3;

% Apply the new scale
ABUval = mean(ABUVo,1);
ABUval = [ABUval(1) ABUval(2) max(ABUVo(:,3))];
ABUVo = (ABUVo - ABUval)*incfactor + ABUval;
ABRval = mean([-0.1883 -0.10615 -1.421;-0.34995 0.22635 -1.8125]);
ABRVo = (ABRVo - ABRval)*incfactor + ABRval;
ABLval = mean([0.1883 -0.10615 -1.421;0.34995 0.22635 -1.8125]);
ABLVo = (ABLVo - ABLval)*incfactor + ABLval;
AWRval = mean([-1.314 -0.1152 0.3634;-1.625 0.2472 0.22]);
AWRVo = (AWRVo - AWRval)*incfactor + AWRval;
AWLval = mean([1.314 -0.1152 0.3634;1.625 0.2472 0.22]);
AWLVo = (AWLVo - AWLval)*incfactor + AWLval;