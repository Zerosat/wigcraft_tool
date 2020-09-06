function val = posign(in)
    if (in ~= 0)
        val = sign(in);
    else
        val = 1;
    end
end