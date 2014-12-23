time = (0:length(posError)-1)*0.1
%%
figure
hold all
k = 8;
plot(posError(k,:));
plot(posErrorLS(k,:))
legend('PosError','LS')

%%
figure
hold all
plot(time, meanPosError);
plot(time, meanPosErrorLS,'r');
%plot(meanPosErrorC,'g');
legend('Tight integration','Least squares')
xlabel('time[s]'); ylabel('\deltap [m]')

%% Single
figure
j = 30
hold all
plot(time, posError(j,:));
plot(time, posErrorLS(j,:),'r')
legend('Tight integration','Least squares')
xlabel('time[s]'); ylabel('\deltap [m]')
title('Single realization, error in p_{mb}^m')

%%
figure
subplot(2,1,1)
hold on
title('Tight');
plot(path.sav_p_MB_M(2,:), path.sav_p_MB_M(1,:))
plot(path.sav_p_MB_M_naveq_c(2,:), path.sav_p_MB_M_naveq_c(1,:),'r');
plot(p_E,p_N,'g')
grid on
legend('p_{true}','p_{naveq}','p_{est}');

subplot(2,1,2)
hold on
grid on
title('Least squares')
plot(path.sav_p_MB_M(2,:), path.sav_p_MB_M(1,:))
plot(path.sav_p_MB_M_naveq_c(2,:), path.sav_p_MB_M_naveq_c(1,:),'r');
plot(p_E_LS, p_N_LS, 'g')
legend('p_{true}','p_{naveq}','p_{est} LS');


%%% Generate other plots
%% Only trajectory
figure
plot(path.sav_p_MB_M(2,:), path.sav_p_MB_M(1,:))
grid on;
title('2D trajectory in m, p_{mb}^m');
xlabel('y[m]'); ylabel('x[m]');


%% Trajectory and naveq
figure
hold on; grid on;
plot(path.sav_p_MB_M(2,:), path.sav_p_MB_M(1,:));
plot(path.sav_p_MB_M_naveq_c(2,:), path.sav_p_MB_M_naveq_c(1,:),'r');
title('2D trajectories in m, p_{mb}^m true path and INS estimate');
legend('True path', 'INS path')
xlabel('y[m]'); ylabel('x[m]');


%% Naveq error
figure;
plot(time,meanPosErrorC,'r');
title('INS error in p_{mb}^m');
xlabel('time[s]'); ylabel('\deltap [m]');

%%
figure
hold on
grid on
plot(time, meanPosError);
plot(time, meanPosErrorLS,'r');
%plot(meanPosErrorC,'g');
legend('Tight integration','Least squares')
title('Error in p_{mb}^m, 5 % error rate, error detection')
xlabel('time[s]'); ylabel('\deltap [m]');
e_ti = meanPosError(end)
e_ls = meanPosErrorLS(end)
