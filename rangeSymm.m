function ang = rangeSymm(in)
    ang = fmod(in,2*pi);
    while ang < 0
        ang = ang + 2*pi;
    end
    while ang > 2*pi
        ang = ang - 2*pi;
    end

    function m = fmod(a, b)
        if a == 0
            m = 0;
        else
            m = mod(a, b) + (b*(sign(a) - 1)/2);
        end
    end
end

