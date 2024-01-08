scheme = "DC_motor_state_feedback_control.zcos";

// Import model
importXcosDiagram(scheme);
// Parameters for the model
ctx = "m = 0.1; r = 0.05; J = 0.5*m*r^2; b = 0.0000095; kt = 0.0187; R = 0.6; L = 0.35/1000; ke = 0.0191;";

// Execut the string to create variables in the workspace
execstr(ctx);

// Compute A, B, C, D matrices
A = [0 1 0; 0 -b/J kt/J; 0 -ke/L -R/L];
B = [0; 0; 1/L];
C = [1 0 0];
D = 0;

// Calculate the controllability matrix
Co = cont_mat(A, B);
// Check if the system is controllable
if rank(Co) == size(A, 1) then
    disp("The system is controllable.");
else
    disp("The system is not controllable.");
end

// Compute state feedback gains using pole placement
K = ppol(A, B, [-5 -20 -100]);

// Find setpoint gain needed
Kr = 1/((C + D*K)*inv(-A+B*K)*B+D);

// Assign context
scs_m.props.context = ctx;
// Simulate
xcos_simulate(scs_m, 4);

// Draw
show_window(1);
subplot(411);
h = plot(theta_out.time, theta_out.values, 'b-', theta_setpoint.time, theta_setpoint.values, 'r--', 'LineWidth',3);
l = legend("Feedback", "Setpoint");
l.font_size = 3;
ax=gca();
set(gca(),"grid",[1 1]);
xlabel('Time [s]', 'font_style', 'times bold', 'font_size', 3);
ylabel('Angular position [rad]', 'font_style', 'times bold', 'font_size', 3);

subplot(412);
h = plot(omega_out.time, omega_out.values, 'b-', 'r--', 'LineWidth',3);
ax=gca();
set(gca(),"grid",[1 1]);
xlabel('Time [s]', 'font_style', 'times bold', 'font_size', 3);
ylabel('Angular speed [rad/s]', 'font_style', 'times bold', 'font_size', 3);

subplot(413);
h = plot(current_out.time, current_out.values, 'b-', 'r--', 'LineWidth',3);
ax=gca();
set(gca(),"grid",[1 1]);
xlabel('Time [s]', 'font_style', 'times bold', 'font_size', 3);
ylabel('Current [A]', 'font_style', 'times bold', 'font_size', 3);

subplot(414);
h = plot(voltage_out.time, voltage_out.values, 'b-', 'r--', 'LineWidth',3);
ax=gca();
set(gca(),"grid",[1 1]);
xlabel('Time [s]', 'font_style', 'times bold', 'font_size', 3);
ylabel('Voltage [V]', 'font_style', 'times bold', 'font_size', 3);


