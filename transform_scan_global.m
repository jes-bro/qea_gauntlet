function [r_g_total] = transform_scan_global(a,b,c,d)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    % place Neato at the origin pointing in the ihat_G direction
    theta = atan(d/c)
    placeNeato(a,b,c,d);
    sub = rossubscriber('/scan');
    r_g_total = zeros(3,360);
   % T_1 = [-0.084 0];
    T_1 = [1 0 -0.084; 0 1 0; 0 0 1];

    % wait a while for the Neato to fall into place
    pause(2);

    % Collect data at the room origin
    scan_message = receive(sub);
    r_1 = scan_message.Ranges(1:end-1);
    theta_1 = deg2rad([0:359]');
    %polarplot(theta_1,r_1,'.')
    %coords = pol2cart(-theta_1,-r_1);
    coords = [r_1.*cos(theta_1),r_1.*sin(theta_1)]';
    quiver(a,b,c,d);
    plot(coords(1,:),coords(2,:),'.');
%     polarplot(-theta_1,-r_1,'*');
%     plot(x, y, "*")

    % vector test
    % R = [1,0];
    % r = [x,y];
    % resultant_vec = R + r;

    % matrices
    R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    T = [1 0 a; 0 1 b; 0 0 1];

    for point = 1:360
        r = [coords(1,point); coords(2,point); 1];
        r_n = T_1 * r;
        plot(r_n(1,:),r_n(2,:));
        r_g = T * R * r_n;
        plot(r_g(1,:),r_g(2,:));
        r_g_total(:,point) = r_g(:, 1);
    end 
    plot(r_g_total(1,:),r_g_total(2,:),'.');
    axis equal 
end

