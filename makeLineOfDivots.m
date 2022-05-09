function z_total = makeLineOfDivots(startPoint,endPoint,numPoints,z_total,a)
    %UNTITLED Summary of this function goes here
    %   Detailed explanation goes here
    rise = endPoint(2) - startPoint(2);
    run = endPoint(1) - startPoint(1);
    big_vec = [run rise];
    little_vec = big_vec / (numPoints - 1);
    current_point = startPoint;
    for i = 1:numPoints
        z_total = makeDivot(current_point,z_total,a);
        current_point = current_point + little_vec;
    end 
end

