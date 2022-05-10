function z_new = makeDivot(positions,Z,a)
    % Add the equation of the sink / source at positions to the 
    % potential field defined by Z at a scale of a
    syms x y
    z_new = a * log(sqrt((x-positions(1)).^2+(y-positions(2)).^2)) + Z;
end