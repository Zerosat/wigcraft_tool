function [in2,in1,in0,inn1,inn2] = get_lagrangian_points(gsize,ix)
    if (ix+2)>gsize 
        if (ix+1)>gsize
            % The node type is #3
            [in2,in1,in0,inn1,inn2] = deal(ix,ix-1,ix-2,ix-3,ix-4);
        else
            % The node type is #2
            [in2,in1,in0,inn1,inn2] = deal(ix+1,ix,ix-1,ix-2,ix-3);
        end
    else
        % The node type is #1
        % Limits (left)
        if (ix-2)<1    
            if (ix-1)<1
                % The node type is #3
                [in2,in1,in0,inn1,inn2] = deal(ix+4,ix+3,ix+2,ix+1,ix);
            else
                % The node type is #2
                [in2,in1,in0,inn1,inn2] = deal(ix+3,ix+2,ix+1,ix,ix-1);
            end
        else
            % The node type is #1
            [in2,in1,in0,inn1,inn2] = deal(ix+2,ix+1,ix,ix-1,ix-2);
        end
    end
end