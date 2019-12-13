function [score, reach_score, mass_score, torque_score, manip_score] = rate(x,mount_point,farm,R,Nx,Ny,Nz)

[arm, mass, torque_limit] = farm(x,mount_point);
num_link = arm.n;
qlimMin = arm.qlim(:,1)';
qlimMax = arm.qlim(:,2)';

% Default zero position
q0 = zeros(1,num_link);

wheel_width = .2;
inter_wheel_width = .7;
wheel_radius = .15;
box_width = inter_wheel_width;
box_length = .35;
box_height = .13;
box_to_front_of_wheel = .7;
box_height_from_ground = .27;

% Workspace dimensions, evaluate more
workspace_x = box_to_front_of_wheel + .5;
workspace_y = inter_wheel_width;
workspace_z = 1.5;

x0 = mount_point(1,4) + .5;
y0 = -workspace_y/2;
z0 = 0;

% Number of cells in each dimension

cell_x = workspace_x / Nx;
cell_y = workspace_y / Ny;
cell_z = workspace_z / Nz;

ws_hits = zeros(Nx,Ny,Nz);

repeats = 0;
count = 0;
outs = 0;
ins = 0;

load = [0; 0; 70; 0; 0; 0];
jacob = zeros(6,num_link,0);
torque = [];
torque_holder = [];
qs = zeros(num_link,1);
Ts = arm.fkine(q0);
manipScore = [];
torque_holder = [];

while repeats < R && outs < R*5
    count = count + 1;
    q = rand(1,num_link) .* (qlimMax - qlimMin) + qlimMin;
    T = arm.fkine(q);
    pos = T.t;
    xn = num2cell(pos(1),cell_x,x0,Nx);
    yn = num2cell(pos(2),cell_y,y0,Ny);
    zn = num2cell(pos(3),cell_z,z0,Nz);
    Ts(count) = T;
    qs(:,count) = q;
    
    if ~(isnan(xn) || isnan(yn) || isnan(zn))
        
        % If the position is within the workspace
        
        if ws_hits(xn,yn,zn) == 0
            repeats = 0;
        else
            repeats = repeats + 1;
        end
        outs = 0;
        ins = ins + 1;
        
        ws_hits(xn,yn,zn) = ws_hits(xn,yn,zn) + 1;
        
        J = arm.jacob0(q);
        manipScore(end+1) = manipRatingFunction(J);
        jacob(:,:,end+1) = J;
        torque(:,end+1) = arm.gravload(q)' + J'*load;
        holder = (torque(:,end).*[0 1/30 1/30 1/30 1/5 1/30 1/5]').^2;
        %holder = (holder ./ torque_limit) .^ 2;
        holder = mean(holder);
        torque_holder(end+1) = holder;
    else
        outs = outs + 1;    
    end
    
%     if repeats == R
%         disp("Repeats")
%     elseif outs == R*5
%         disp("Outs")
%     end
end

% Percent workspace reached
hits = sum(sum(sum(ws_hits > 0)));
max = Nx*Ny*Nz;
reach_score = hits/max;

% max_mass = num_link*13/7;
% 
% mass_score = -(mass/max_mass)^5;

mass_score = -sum(x);

torque_score = -mean(torque_holder);

manip_score = mean(manipScore);

W = [1 1 .5 1];

score = reach_score*W(1) + mass_score*W(2) + torque_score*W(3) + manip_score*W(4);
if isnan(score)
    score = -10;
end
score = -score;
end

function N = num2cell(num,cell,p0,Nmax) 
a = num-p0;
if a < 0
    N = NaN;
    return
end
N = floor(a/cell) + 1;
if N > Nmax
    N = NaN;
end
end

function score = manipRatingFunction(jacob)

%J = jacob(1:3,:);
J = jacob;

if cond(J*J') > 1000
    score = 0;
    return
else
    JJ = J*J';
    d = det(JJ);
    score = 2*d-d^2;
end

end