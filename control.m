A1 = 5; A2 = 4; R1 = 3; R2 = 5;
cc_eqn = [A1*A2*R1*R2, A1*R1 + A2*R2 + A1*R2, 1];

%1,2. transfer functions
Qo_Qin = tf([1], cc_eqn)
Q1_Qin = tf([A2*R2, 1], cc_eqn)
H1_Qin = tf([A2*R2, R1+R2], cc_eqn)
H2_Qin = tf([R2], cc_eqn)

%3. study system stability
poles = roots(cc_eqn)

%visualize
figure;
pzmap(Qo_Qin);
grid on;
title('Pole-Zero Plot of the System');


%4. step responses
figure;
step(Qo_Qin);
title('Step Response: Qo/Qin');

figure;
step(Q1_Qin);
title('Step Response: Q1/Qin');

figure;
step(H1_Qin);
title('Step Response: H1/Qin');

figure;
step(H2_Qin);
title('Step Response: H2/Qin');

%5,6. track hd
t = 0:0.1:100;
Hd_H2 = feedback(H2_Qin, 1)
hd = 5;
figure;
[y_H2, t] = step(Hd_H2, t);
y_H2 = hd * y_H2;


% 7. get h2 transient response
step(5*Hd_H2);
stepinfo(5*Hd_H2)

% Compute H1 response
Qin = (hd - y_H2);
y_H1 = lsim(H1_Qin, Qin, t);
figure;
plot(t, y_H2, 'b', t, y_H1, 'r--', t, hd*ones(size(t)), 'k:');
legend('h_2', 'h_1', 'h_d');
title('Response to h_d = 5m (P-Control)');
grid on;

% 8,9. different kp values
hd = 5;
Kp_values = [1, 10, 100];
figure;
hold on;
for Kp = Kp_values
    Hd_H2 = feedback(Kp * H2_Qin, 1);
    [y, t] = step(Hd_H2, t);
    y = hd * y;
    ess = hd - y(end);
    plot(t, y, 'DisplayName', sprintf('Kp = %d (e_{ss} = %.5f)', Kp, ess));
end
title('Effect of K_p on Tracking and e_{ss}');
legend; grid on; hold off;


% 10. PID control
Kp =20;   % Proportional gain
KI = 1;   % Integral gain
Kd = 150;  % Derivative gain

% PID controller transfer function
C = tf([Kd Kp KI], [1 0]);  % Represents Kd*s + Kp + KI/s

Hd_H2 = feedback(C * H2_Qin, 1)

figure;
step(6 * Hd_H2);  % Scale for 6m setpoint
stepinfo(6 * Hd_H2)
title(sprintf('PID Control: Kp=%.2f, KI=%.2f, Kd=%.2f', Kp, KI, Kd));
xlabel('Time (s)');
ylabel('Height (m)');
grid on;