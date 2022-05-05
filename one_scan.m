sub = rossubscriber('/scan');

% place Neato at the origin pointing in the ihat_G direction
a = 0;
b = 0;
c = 1;
d = 0;
theta = tan(d/c);
placeNeato(a, b,c,d);
r_g_total = zeros(3,360);

% wait a while for the Neato to fall into place
pause(2);

% Collect data at the room origin
scan_message = receive(sub);
r_1 = scan_message.Ranges(1:end-1);
theta_1 = deg2rad([0:359]');
[x,y] = pol2cart(-theta_1,-r_1);
%polarplot(-theta_1,-r_1,'*');

% vector test
% R = [1,0];
% r = [x,y];
% resultant_vec = R + r;

% matrices
R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
T = [1 0 -a; 0 1 -b; 0 0 1];

for point = 1:length(x)
    r = [x(point); y(point); 1];
    r_g = T * R * r;
    r_g_total(:,point) = r_g(:, 1);
    hold on
end 
plot(r_g_total(1,:),r_g_total(2,:),'.');
axis equal 