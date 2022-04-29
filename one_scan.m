sub = rossubscriber('/scan');

% place Neato at the origin pointing in the ihat_G direction
placeNeato(0,0,1,0)

% wait a while for the Neato to fall into place
pause(2);

% Collect data at the room origin
scan_message = receive(sub);
r_1 = scan_message.Ranges(1:end-1);
theta_1 = deg2rad([0:359]');