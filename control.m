A1 = 5; A2 = 4; R1 = 3; R2 = 5;
cc_eqn = [A1*A2*R1*R2, A1*R1 + A2*R2 + A1*R2, 1];

%transfer functions
Qo_Qin = tf([1], cc_eqn);
Q1_Qin = tf([A2*R2, 1], cc_eqn);
H1_Qin = tf([A2*R2, R1+R2], cc_eqn);
H2_Qin = tf([R2], cc_eqn);

%study system stability
poles = roots(cc_eqn);

%step responses
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

%5. track hd
t = 0:0.1:100;
Hd_H2 = feedback(H2_Qin, 1);
hd = 5;
figure;
[y_H2, t] = step(Hd_H2, t);
y_H2 = hd * y_H2;

% Compute H1 response
Qin = (hd - y_H2);
y_H1 = lsim(H1_Qin, Qin, t);
figure;
plot(t, y_H2, 'b', t, y_H1, 'r--', t, hd*ones(size(t)), 'k:');
legend('h_2', 'h_1', 'h_d');
title('Response to h_d = 5m (P-Control)');
grid on;

% different kp values
hd = 5;
Kp_values = [1, 10, 100];
figure; hold on;
for Kp = Kp_values
Hd_H2 = feedback(Kp * H2_Qin, 1);
[y, t] = step(Hd_H2, t);
y = hd * y;
ess = hd - y(end);
plot(t, y, 'DisplayName', sprintf('Kp = %d (e_{ss} = %.3f)', Kp, ess));
end
title('Effect of K_p on Tracking and e_{ss}');
legend; grid on;