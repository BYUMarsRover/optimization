function [arm, mass, torque_limit] = full7DOF(x,mount)
% arm = offset3link_full(x,qlimMin,qlimMax)

% *******
% LENGTHS
% *******

qlimMin = [0, -pi/2, -pi/1.3, -pi/1.3, -pi, -pi/2, -pi];
qlimMax = [1.5, pi/2, pi/1.3, pi/1.3, pi, pi/2, pi];

% Note: the base frame is oriented such that the x direction is towards the
% equipment servicing cabinet, away from the rover. z is positive vertical
% direction, y is to the left if facing the front of the rover.

lin_ac_min = qlimMin(1); % Distance below base that the linear actuator can reach
lin_ac_max = qlimMax(1); % Distance above the base that the linear actuator can reach

link_1_x = x(1); % Horizontal length of first link (the link with the offset in it)
link_1_z = x(2); % Vertical offset of the first link

link_2_x = x(3); % Horizontal length of the second link (between two z-facing motors)

link_3_x = x(4); % Distance from z oriented motor to the first wrist motor (first axial motor)

link_4_x = x(5); % Distance from the first axial motor to the z oriented motor on the wrist

link_5_x = x(6); % Distance from the z oriented wrist motor the final, axial motor

link_6_x = x(7); % Distance from last motor to the gripper

% *******
% WEIGHTS
% *******

% All weights given in kg

link_density = .16 / 1.5 * 3.28 * .453; % Define linear density of each link, kg/m

lin_ac_mass = 4; % Includes the actuator and the screw?

motor_1_mass = 2; % First z oriented motor, attached to the linear actuator

motor_2_mass = 2; % Second z oriented motor, attached after the offset link

motor_3_mass = 2; % Last z oriented motor, attached at the end of the arm

motor_4_mass = 1; % First wrist motor, axial

motor_5_mass = 1; % Second wrist motor, z oriented

motor_6_mass = .5; % Final wrist motor, axial

% all parameters are in SI units: m, radians, kg, kg.m2, N.m, N.m.s etc.


rx0 = 0;
ry0 = 0;
rz0 = 0;

total_mass0 = lin_ac_mass;

% Prismatic Link: From base to upper extent of prismatic motor
L0 = Link('theta', 0, ...
    'a', 0, ...   % link length in x (Dennavit-Hartenberg notation)
    'alpha', 0, ...        % link twist (Dennavit-Hartenberg notation)
    'offset', lin_ac_min, ...       % offset for prismatic joint, note that qlim-min for prismatic joint cannot be negative, but the offset can
    'I', [0, 0, 0, 0, 0, 0], ... % inertia tensor of link with respect to center of mass I = [L_xx, L_yy, L_zz, L_xy, L_yz, L_xz]
    'r', [rx0, ry0, rz0], ...       % distance of ith origin to center of mass [x,y,z] in link reference frame
    'm', total_mass0, ...               % mass of link
    'Jm', 0, ...         % actuator inertia 
    'G', 1, ...        % gear ratio
    'B', 0, ...         % actuator viscous friction coefficient (measured at the motor)
    'Tc', [0 0], ... % actuator Coulomb friction coefficient for direction [-,+] (measured at the motor)
    'qlim', [0 lin_ac_max - lin_ac_min] ); % minimum and maximum joint angle

total_mass1 = (link_1_z + link_1_x) * link_density + motor_1_mass;
rx1 = ((-link_1_x/2 * link_1_x * link_density) + (link_1_z * link_density * -link_1_x) + (motor_1_mass * -link_1_x)) / total_mass1;
ry1 = 0;
rz1 = ((-link_1_z / 2 * link_1_z * link_density) + (link_1_x * link_density * 0) + (motor_1_mass * -link_1_z)) / total_mass1;

L1 = Link('d', link_1_z, ...
    'a', link_1_x, ...   % link length in x (Dennavit-Hartenberg notation)
    'alpha', 0, ...        % link twist (Dennavit-Hartenberg notation)
    'offset', 0, ...       % offset for prismatic joint, note that qlim-min for prismatic joint cannot be negative, but the offset can
    'I', [0, 0, 0, 0, 0, 0], ... % inertia tensor of link with respect to center of mass I = [L_xx, L_yy, L_zz, L_xy, L_yz, L_xz]
    'r', [rx1, ry1, rz1], ...       % distance of ith origin to center of mass [x,y,z] in link reference frame
    'm', total_mass1, ...               % mass of link
    'Jm', 0, ...         % actuator inertia 
    'G', 1, ...        % gear ratio
    'B', 0, ...         % actuator viscous friction coefficient (measured at the motor)
    'Tc', [0 0], ... % actuator Coulomb friction coefficient for direction [-,+] (measured at the motor)
    'qlim', [qlimMin(2) qlimMax(2)] ); % minimum and maximum joint angle

total_mass2 = link_2_x * link_density +  motor_2_mass;
rx2 = ((-link_2_x/2 * link_2_x * link_density) + (motor_2_mass * -link_2_x)) / total_mass2;
ry2 = 0;
rz2 = 0;

L2 = Link('d', 0, ...
    'a', link_2_x, ...   % link length in x (Dennavit-Hartenberg notation)
    'alpha', 0, ...        % link twist (Dennavit-Hartenberg notation)
    'offset', 0, ...       % offset for prismatic joint, note that qlim-min for prismatic joint cannot be negative, but the offset can
    'I', [0, 0, 0, 0, 0, 0], ... % inertia tensor of link with respect to center of mass I = [L_xx, L_yy, L_zz, L_xy, L_yz, L_xz]
    'r', [rx2, ry2, rz2], ...       % distance of ith origin to center of mass [x,y,z] in link reference frame
    'm', total_mass2, ...               % mass of link
    'Jm', 0, ...         % actuator inertia 
    'G', 1, ...        % gear ratio
    'B', 0, ...         % actuator viscous friction coefficient (measured at the motor)
    'Tc', [0 0], ... % actuator Coulomb friction coefficient for direction [-,+] (measured at the motor)
    'qlim', [qlimMin(3) qlimMax(3)] ); % minimum and maximum joint angle

total_mass3 = link_3_x * link_density +  motor_3_mass;
rx3 = 0;
ry3 = 0;
rz3 = ( (-link_3_x * link_density * link_3_x / 2) + (-link_3_x * motor_3_mass) ) / total_mass3;

L3 = Link('d', 0, ...
    'a', 0, ...   % link length in x (Dennavit-Hartenberg notation)
    'alpha', pi/2, ...        % link twist (Dennavit-Hartenberg notation)
    'offset', pi/2, ...       % offset for prismatic joint, note that qlim-min for prismatic joint cannot be negative, but the offset can
    'I', [0, 0, 0, 0, 0, 0], ... % inertia tensor of link with respect to center of mass I = [L_xx, L_yy, L_zz, L_xy, L_yz, L_xz]
    'r', [rx3, ry3, rz3], ...       % distance of ith origin to center of mass [x,y,z] in link reference frame
    'm', total_mass3, ...               % mass of link
    'Jm', 0, ...         % actuator inertia 
    'G', 1, ...        % gear ratio
    'B', 0, ...         % actuator viscous friction coefficient (measured at the motor)
    'Tc', [0 0], ... % actuator Coulomb friction coefficient for direction [-,+] (measured at the motor)
    'qlim', [qlimMin(4) qlimMax(4)] ); % minimum and maximum joint angle

total_mass4 = link_4_x * link_density +  motor_4_mass;
rx4 = 0;
ry4 = ((link_4_x * link_density * link_4_x / 2) + (link_4_x * motor_4_mass)) / total_mass4;
rz4 = 0;

L4 = Link('d', link_4_x+link_3_x, ...
    'a', 0, ...   % link length in x (Dennavit-Hartenberg notation)
    'alpha', -pi/2, ...        % link twist (Dennavit-Hartenberg notation)
    'offset', 0, ...       % offset for prismatic joint, note that qlim-min for prismatic joint cannot be negative, but the offset can
    'I', [0, 0, 0, 0, 0, 0], ... % inertia tensor of link with respect to center of mass I = [L_xx, L_yy, L_zz, L_xy, L_yz, L_xz]
    'r', [rx4, ry4, rz4], ...       % distance of ith origin to center of mass [x,y,z] in link reference frame
    'm', total_mass4, ...               % mass of link
    'Jm', 0, ...         % actuator inertia 
    'G', 1, ...        % gear ratio
    'B', 0, ...         % actuator viscous friction coefficient (measured at the motor)
    'Tc', [0 0], ... % actuator Coulomb friction coefficient for direction [-,+] (measured at the motor)
    'qlim', [qlimMin(5) qlimMax(5)] ); % minimum and maximum joint angle

total_mass5 = link_5_x * link_density +  motor_5_mass;
rx5 = 0;
ry5 = 0;
rz5 = ((link_5_x * link_density * link_5_x / 2) + (link_5_x * motor_5_mass)) / total_mass5;

L5 = Link('d', 0, ...
    'a', 0, ...   % link length in x (Dennavit-Hartenberg notation)
    'alpha', pi/2, ...        % link twist (Dennavit-Hartenberg notation)
    'offset', 0, ...       % offset for prismatic joint, note that qlim-min for prismatic joint cannot be negative, but the offset can
    'I', [0, 0, 0, 0, 0, 0], ... % inertia tensor of link with respect to center of mass I = [L_xx, L_yy, L_zz, L_xy, L_yz, L_xz]
    'r', [rx5, ry5, rz5], ...       % distance of ith origin to center of mass [x,y,z] in link reference frame
    'm', total_mass5, ...               % mass of link
    'Jm', 0, ...         % actuator inertia 
    'G', 1, ...        % gear ratio
    'B', 0, ...         % actuator viscous friction coefficient (measured at the motor)
    'Tc', [0 0], ... % actuator Coulomb friction coefficient for direction [-,+] (measured at the motor)
    'qlim', [qlimMin(6) qlimMax(6)] ); % minimum and maximum joint angle

total_mass6 = link_6_x * link_density +  motor_6_mass;
rx6 = 0;
ry6 = 0;
rz6 = ((link_6_x * link_density * link_6_x / 2) + (link_6_x * motor_6_mass)) / total_mass6;

L6 = Link('d', link_5_x + link_6_x, ...
    'a', 0, ...   % link length in x (Dennavit-Hartenberg notation)
    'alpha', 0, ...        % link twist (Dennavit-Hartenberg notation)
    'offset', 0, ...       % offset for prismatic joint, note that qlim-min for prismatic joint cannot be negative, but the offset can
    'I', [0, 0, 0, 0, 0, 0], ... % inertia tensor of link with respect to center of mass I = [L_xx, L_yy, L_zz, L_xy, L_yz, L_xz]
    'r', [rx6, ry6, rz6], ...       % distance of ith origin to center of mass [x,y,z] in link reference frame
    'm', total_mass6, ...               % mass of link
    'Jm', 0, ...         % actuator inertia 
    'G', 1, ...        % gear ratio
    'B', 0, ...         % actuator viscous friction coefficient (measured at the motor)
    'Tc', [0 0], ... % actuator Coulomb friction coefficient for direction [-,+] (measured at the motor)
    'qlim', [qlimMin(7) qlimMax(7)] ); % minimum and maximum joint angle

base = mount;

arm = SerialLink([L0,L1,L2,L3,L4,L5,L6],'base',base);
mass = total_mass0 + total_mass1 + total_mass2 + total_mass3 + total_mass4 + total_mass5 + total_mass6;
torque_limit = [150; 30; 30; 30; 2; 30; 2];
end