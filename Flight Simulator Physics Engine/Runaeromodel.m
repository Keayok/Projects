clc
close all
clear

% Intialise the intial condition values 
longitude0 = deg2rad(-79.625335);
latitude0 = deg2rad(43.676856);
altitude0 = 7224;
phi0 = 0;
theta0 = 0.1;
psi0 = 0;
U0 = 84.9905;
V0 = 0;
W0 = 1.2713;
P0 = 0;
Q0 = 0.0149;
R0 = 0;

%Put the initial condition values into an matrix that can be input into the
%the integrator block in the simulink file 
init = [longitude0;latitude0;altitude0;phi0;theta0;psi0;U0;V0;W0;P0;Q0;R0];

% Initialise the input values for the Throttle (1 and 2), Aileron, Rudder
% and Elevator. Put into the constant block 
deltaT10 = 0.0821;
deltaT20 = 0.0821;
deltaA0 = 0;
deltaE0 = -0.1780;
deltaR0 = 0;
% Put into the constant block U_0
U_0 = [deltaT10;deltaT20;deltaA0;deltaE0;deltaR0];

% Time to run the sumulation
TF = 50;

% Set parameters that adjust Joystick input. Can be seen in Gain blocks in Input subsystem 
maxAileronDeflection = 25*(pi/180); 
maxElevatorDeflection = 10*(pi/180);
maxRudderDeflection = 30*(pi/180);

%Runs the simulation 
out = sim("AircraftModel.slx");

%Access to the arrays of data after simulation is done
t = out.simX.Time;

x1 = out.simX.Data(:,1);
x2 = out.simX.Data(:,2);
x3 = out.simX.Data(:,3);
x4 = out.simX.Data(:,4);
x5 = out.simX.Data(:,5);
x6 = out.simX.Data(:,6);
x7 = out.simX.Data(:,7);
x8 = out.simX.Data(:,8);
x9 = out.simX.Data(:,9);
x10 = out.simX.Data(:,10);
x11 = out.simX.Data(:,11);
x12 = out.simX.Data(:,12);


%plot results
figure 
subplot(4,3,1)
plot(t,x1)
legend('longitude')
title(x1,'Longitude')
ylabel('Angle(rads)')
xlabel('Time(s)')

grid on
subplot(4,3,2)
plot(t,x2)
legend('latitude')
title(x2,'Latitiude')
ylabel('Angle(rads)')
xlabel('Time(s)')

grid on
subplot(4,3,3)
plot(t,x3)
legend('h(Altitude)')
title(x3,'Altitude')
ylabel('(m)')
xlabel('Time(s)')

grid on
subplot(4,3,4)
plot(t,x4)
legend('phi')
title(x4,'Pitch Angle')
ylabel('Angle(rads)')
xlabel('Time(s)')


grid on
subplot(4,3,5)
plot(t,x5)
legend('theta')
title(x5,'Roll Angle')
ylabel('Angle(rads)')
xlabel('Time(s)')

grid on
subplot(4,3,6)
plot(t,x6)
legend('psi')
title(x6,'Yaw Angle')
ylabel('Angle(rads)')
xlabel('Time(s)')

grid on
subplot(4,3,7)
plot(t,x7)
legend('U')
title(x7,'U-Velocity')
ylabel('(m/s)')
xlabel('Time(s)')

grid on
subplot(4,3,8)
plot(t,x8)
legend('V')
title(x8,'V-Velocity')
ylabel('(m/s)')
xlabel('Time(s)')

grid on
subplot(4,3,9)
plot(t,x9)
legend('W')
title(x9,'W-Velcocity')
ylabel('(m/s)')
xlabel('Time(s)')

grid on
subplot(4,3,10)
plot(t,x10)
legend('P')
title(x10,'Roll Rate')
ylabel('(rad/s)')
xlabel('Time(s)')

grid on
subplot(4,3,11)
plot(t,x11)
legend('Q')
title(x11,'Pitch Rate')
ylabel('(rad/s)')
xlabel('Time(s)')

grid on
subplot(4,3,12)
plot(t,x12)
legend('R')
title(x12,'Yaw Rate')
ylabel('(rad/s)')
xlabel('Time(s)')
grid on
