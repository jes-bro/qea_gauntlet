function z_new = makeDivot(positions,Z,a)
    syms x y
    z_new = a * log(sqrt((x-positions(1)).^2+(y-positions(2)).^2)) + Z;
end