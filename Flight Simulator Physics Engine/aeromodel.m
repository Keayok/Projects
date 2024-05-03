function A_dot = aeromodel(X,inputs)

%% VARIABLES
longitude = X(1);
latitude = X(2);
h = X(3);
phi = X(4);
theta = X(5);
psi = X(6);
U = X(7);
V = X(8);
W = X(9);
P = X(10);
Q = X(11);
R = X(12);

%% INPUTS
delta_T_1 = inputs(1); % Engine 1 throttle angle
delta_T_2 = inputs(2); % Engine 2 throttle angle
delta_A = inputs(3); % Aileron deflection angle
delta_E = inputs(4); % Elevator deflection angle
delta_R = inputs(5); % Rudder deflection angle

%% CONSTANTS
                 % Mass 
 m= 120000;
               % Inertia_Matrix 
J = m*[40.07,0,-2.0923; 0, 64, 0; -2.0923, 0, 99.92];
             % Mean aerodynamic chord
c_bar = 6.6;
     % Coordinates of aircrafts center of mass
cm = [0.23*c_bar; 0; 0.1*c_bar];

    % Coordinates of aircrafts aerodynamic center 
% ac = [0.12*c_bar; 0; 0];

     % Coordinates of engine 1 thrust application point
apt1 = [0;-7.94;-1.9];
      % Coordinates of engine 2 thrust application point 
apt2 = [0;7.94;-1.9];
    % Wing platform area
S = 260;
    % Wing platform area of tail
S_t = 64;
   % Generalized length of wings
l = 6.6;
   % Distance between the aircraft cm and the ac of the tail
l_t = 24.8;
  % Angle of attack when lift is zero
alpha_0 = deg2rad(-11.5);
   % Air Density
Rho = 1.225;
   % Acceleration due to gravity
g = 9.81;
   % Earth equitorial radius 
a = 6378137;  
  % Earths eccentricity 
e = 0.081819190842622;   
%% AERODYNAMCIS 
% Airspeed
V_T = sqrt((U^2) + (V^2) + (W^2));
%Dynamic Pressure
q_bar = 0.5*Rho*(V_T^2);

%Angle of Attack
alpha = atan(W/U);
% Sideslip angle
beta_ = asin(V/V_T);

% Downwash angle of the wings
eppsilon = 0.25*(alpha-alpha_0);

% Angle of attack of the tail
alpha_t = alpha - eppsilon + delta_E + 1.3*Q*(l_t/V_T);

% Aerodynamic Coefficients 
C_L_wb = 5.5*(alpha - alpha_0);
C_L_t = 3.1*(S_t/S)*alpha_t;
% Lift Coefficient 
C_L = C_L_wb + C_L_t;

% Drag Coefficient 
C_D = 0.13 + 0.07*((5.5*alpha + 0.654)^2);

% Crosswind Coefficient 
C_C = -1.6*beta_ + 0.24*delta_R;

% Rolling moment Coefficient
C_l = -1.4*beta_ - 11*(l/V_T)*P + 5*(l/V_T)*R - 0.6*delta_A + 0.22*delta_R;

% Pitching moment Coefficient
C_m = -0.59 - 3.1*((S_t*l_t)/(S*l))*(alpha-eppsilon)-(4.03)*((S_t*(l_t^2))/(S*(l^2)))*(l/V_T)*(Q) - (3.1)*((S_t*l_t)/(S*l))*delta_E;

% Yawing moment coeficient 
C_n = (1 - 3.8179*alpha)*beta_ + 1.7*(l/V_T)*P - 11.5*(l/V_T)*R - 0.63*delta_R;

% Aerodynamic Forces 

D = q_bar*S*C_D; 
C = q_bar*S*C_C; 
L = q_bar*S*C_L;

% Resultant Forces
X_A = -D*cos(alpha)*cos(beta_) + C*cos(alpha)*sin(beta_)+L*sin(alpha);
Y_A = -D*sin(beta_) - C*cos(beta_);
Z_A = -D*sin(alpha)*cos(beta_) + C*sin(alpha)*sin(beta_)-L*cos(alpha);

Vector_forces = [X_A ; Y_A ; Z_A];

% Aerodynamic moments for simulink aswell
l_prime_A = q_bar*c_bar*S*C_l;
m_prime_A = q_bar*c_bar*S*C_m;
n_prime_A = q_bar*c_bar*S*C_n;

% Calculation 
Vector_moment_prime = [l_prime_A; m_prime_A; n_prime_A];
Vector_constants = [0.11*c_bar; 0 ; 0.1*c_bar];

Actual_Aero_Moments  = Vector_moment_prime +  cross(Vector_forces,Vector_constants);

l_A = Actual_Aero_Moments(1);
m_A = Actual_Aero_Moments(2);
n_A = Actual_Aero_Moments(3);

%% ENGINE EQUATIONS 
% Engine forces 
XT1 = delta_T_1*m*g;
XT2 = delta_T_2*m*g;
YT = 0;
ZT = 0;
XT = XT1 + XT2;

% Engine moments 
Engine1_moment1 = [XT1;0;0];
Engine1_moment2 = cm - apt1;
Engine2_moment1 = [XT2;0;0];
Engine2_moment2 = cm - apt2;

Engine1_total_moment = cross(Engine1_moment2,Engine1_moment1);
Engine2_total_moment = cross(Engine2_moment2,Engine2_moment1);

l_Ti = Engine1_total_moment(1) + Engine2_total_moment(1);
m_Ti = Engine1_total_moment(2) + Engine2_total_moment(2);
n_Ti = Engine1_total_moment(3) + Engine2_total_moment(3);

%% FORCE EQUATIONS 

U_dot = R*V - Q*W - g*sin(theta) + (XT+X_A)/m;
V_dot  = -R*U + P*W + g*sin(phi)*cos(theta) + (YT+Y_A)/m;
W_dot  = Q*U - P*V + g*cos(phi)*cos(theta) + (ZT+Z_A)/m; 

%% MOMENT EQUATIONS

% Moment equations

J_x = J(1,1);
J_y = J(2,2);
J_z = J(3,3);
J_xz = J(3,1);

Gamma = J_x*J_z - (J_xz^2);

Pdot = (J_xz*(J_x - J_y + J_z)*P*Q-(J_z*(J_z-J_y) + (J_xz^2))*Q*R+J_z*(l_A+l_Ti)+J_xz*(n_A +n_Ti))/Gamma;
Qdot = ((J_z-J_x)*P*R-J_xz*((P^2)-(R^2))+ m_A + m_Ti)/J_y;
Rdot = (((J_x-J_y)*J_x+J_xz^2)*P*Q-J_xz*(J_x-J_y+J_z)*Q*R+J_xz*(l_A+l_Ti)+J_x*(n_A+n_Ti))/Gamma;


%% KINEMATIC EQUATIONS

phi_dot = P + (tan(theta))*(Q*sin(phi) + R*cos(phi));
theta_dot = Q*cos(phi) - R*sin(phi);
psi_dot = (Q*sin(phi) + R*cos(phi))/cos(theta);

%% NAVIGATION EQUATIONS 

P_ndot = U*cos(theta)*cos(psi)+V*(-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi))+W*(sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi));
P_edot = U*cos(theta)*sin(psi)+V*(cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi))+W*(-sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi));
h_dot = U*sin(theta)-V*sin(phi)*cos(theta)-W*cos(phi)*cos(theta);

%% GEODECTIC COORDINATES EQUATIONS 

% Meridian radius 
M = a*(1-(e^2))/sqrt((1-(e^2)*sin(phi)^2)^3);  

% Prime vertical Radius 
N = a/sqrt(1-(e^2)*sin(phi)^2);   

latitude_dot = P_ndot/(M + h);
longitude_dot = P_edot/((N + h)*cos(latitude));

A_dot = [longitude_dot;latitude_dot;h_dot;phi_dot;theta_dot;psi_dot;U_dot;V_dot;W_dot;Pdot;Qdot;Rdot];
end 

