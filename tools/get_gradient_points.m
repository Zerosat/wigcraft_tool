function [in1,in0,inn1] = get_gradient_points(gsize,ix)
    if (ix+1)>gsize
        % The node type is #3
        [in1,in0,inn1] = deal(ix,ix-1,ix-2);
    else
        if (ix-1)<1
            % The node type is #3
            [in1,in0,inn1] = deal(ix+2,ix+1,ix);
        else
            % The node type is #2
            [in1,in0,inn1] = deal(ix+1,ix,ix-1);
        end
    end
end