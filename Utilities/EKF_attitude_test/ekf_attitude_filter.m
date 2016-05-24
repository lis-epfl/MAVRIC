%% Simulated sensors
clc

plotNormMag = 0;
plotMag = 0;

plotNormAcc = 0;
plotAcc = 0;

plotNormGyro = 0;
plotGyro = 0;
plotAttCommand = 0;

plotNormBias = 0;
plotBias = 0;

plotQuat = 0;
plotEuler = 0;
plotQuatFinal = 1;
plotBiasFinal = 1;
plotPredictCorr = 0;
plotEulerEKF = 1;

%% Simulated sensors
time = (0:0.04:100)';

biasPerfect = zeros(size(time,1),3);
for t = 1:size(time,1)
    biasPerfect(t,:) = biasPerfect(t,:) + [0.0035 -0.003 0.003] * t/size(time,1);
end
bias = awgn(biasPerfect,100); %100

axisAttCmd = 1;
attCommand = zeros(size(time,1),3);
attCommand(1:floor(size(time,1)/2)+1,axisAttCmd) = pi/4;
attCommand(floor(size(time,1)/2):end,axisAttCmd) = pi/4*sin((time(floor(size(time,1)/2):end)-time(floor(size(time,1)/2)))*pi/12+pi/2);

%gyroInte = repmat([0 0 0],size(time,1),1);
gyroInte = zeros(size(time,1),3);
%gyroInte(floor(size(time,1)/2):end,axisAttCmd) = 0.00025*sin((time(floor(size(time,1)/2):end)-time(floor(size(time,1)/2)))*pi/12);
gyroInte(floor(size(time,1)/2):end,axisAttCmd) = pi/4*pi/12*cos((time(floor(size(time,1)/2):end)-time(floor(size(time,1)/2)))*pi/12+pi/2);


gyroPerfect = repmat([0 0 0],size(time,1),1) + biasPerfect;
gyro = awgn(gyroInte,100) + bias;


q = zeros(size(time,1),4);
q(1,:) = [1 0 0 0];
qNoise = zeros(size(time,1),4);
qNoise(1,:) = [1 0 0 0];

qGyro = [zeros(size(time,1),1) gyroInte];
qGyroNoise = [zeros(size(time,1),1) gyro];
for t = 2:size(time,1)
    q(t,:) = q(t-1,:) + qmult(q(t-1,:),qGyro(t,:)/2)*(time(t)-time(t-1));
    q(t,:) = q(t,:)/norm(q(t,:));
    qNoise(t,:) = qNoise(t-1,:) + qmult(qNoise(t-1,:),qGyroNoise(t,:)/2)*(time(t)-time(t-1));
    qNoise(t,:) = qNoise(t,:)/norm(qNoise(t,:));
end

euler = zeros(size(time,1),3);
for t = 1:size(time,1)
    [euler(t,1), euler(t,2), euler(t,3)] = quat2euler(q(t,:));
end

mag_global = [cos(deg2rad(63)) 0 sin(deg2rad(63))];
%magPerfect = repmat(mag_global,size(time,1),1);
magPerfect = zeros(size(time,1),3);
magPerfect(1,:) = mag_global;
for t = 2:size(time,1)
    qt = q(t,:);
    
    rotMatrix = [qt(1)^2+qt(2)^2-qt(3)^2-qt(4)^2 2*(qt(2)*qt(3)+qt(1)*qt(4)) 2*(qt(1)*qt(3) - qt(2)*qt(4))
        2*(qt(2)*qt(3) - qt(1)*qt(4)) qt(1)^2-qt(2)^2+qt(3)^2-qt(4)^2 2*(qt(3)*qt(4) + qt(1)*qt(2))
        2*(qt(2)*qt(4) + qt(1)*qt(3)) 2*(qt(1)*qt(2) - qt(3)*qt(4)) qt(1)^2-qt(2)^2+qt(3)^2-qt(4)^2];
    
    magPerfect(t,:) = (rotMatrix*magPerfect(1,:)')';
    magPerfect(t,:) = magPerfect(t,:)/norm(magPerfect(t,:));
end
mag = awgn(magPerfect,40); %40

% angleRot = 0;%pi/2;
% rotMatrix = [1 0 0
%     0 cos(angleRot) -sin(angleRot);
%     0 sin(angleRot) cos(angleRot)];
acc_global = [0 0 -1];%(rotMatrix*[0 0 -1]')';
%accPerfect = repmat(acc_global,size(time,1),1);
accPerfect = zeros(size(time,1),3);
accPerfect(1,:) = acc_global;
for t = 2:size(time,1)
    qt = q(t,:);
    
    rotMatrix = [qt(1)^2+qt(2)^2-qt(3)^2-qt(4)^2 2*(qt(2)*qt(3)+qt(1)*qt(4)) 2*(qt(1)*qt(3) - qt(2)*qt(4))
        2*(qt(2)*qt(3) - qt(1)*qt(4)) qt(1)^2-qt(2)^2+qt(3)^2-qt(4)^2 2*(qt(3)*qt(4) + qt(1)*qt(2))
        2*(qt(2)*qt(4) + qt(1)*qt(3)) 2*(qt(1)*qt(2) - qt(3)*qt(4)) qt(1)^2-qt(2)^2+qt(3)^2-qt(4)^2];
    accPerfect(t,:) = (rotMatrix*accPerfect(1,:)')';
    accPerfect(t,:) = accPerfect(t,:)/norm(accPerfect(t,:));
end
acc = awgn(accPerfect,40); %40

if plotNormMag == 1
    normMag = zeros(size(time,1),1);
    for i = 1:size(time,1)
        normMag(i) = norm(mag(i,:));
    end
    figure
    plot(time,normMag)
    title('Norm compass')
end

if plotMag == 1
    figure
    hold on
    plot(time,mag(:,1))
    plot(time,mag(:,2),'g')
    plot(time,mag(:,3),'r')
    %axis([time(1) time(end) -1 1])
    legend('x','y','z')
    title('Compass')
end

if plotNormAcc == 1
    normAcc = zeros(size(time,1),1);
    for i = 1:size(time,1)
        normAcc(i) = norm(acc(i,:));
    end
    figure
    plot(time,normAcc)
    title('Norm accelerometer')
end

if plotAcc == 1
    figure
    hold on
    plot(time,acc(:,1))
    plot(time,acc(:,2),'g')
    plot(time,acc(:,3),'r')
    axis([time(1) time(end) -1.5 1.5])
    legend('x','y','z')
    title('Accelerometer')
end

if plotAttCommand == 1
    figure
    hold on
    plot(time,attCommand(:,1))
    plot(time,attCommand(:,2),'r')
    plot(time,attCommand(:,3),'g')
    legend('x','y','z')
    title('Attitude command')
end

if plotNormGyro == 1
    normGyro = zeros(size(time,1),1);
    for i = 1:size(time,1)
        normGyro(i) = norm(gyro(i,:));
    end
    figure
    plot(time,normGyro)
    title('Norm gyroscope')
end

if plotGyro == 1
    figure
    hold on
    plot(time,gyro(:,1))
    plot(time,gyro(:,2),'g')
    plot(time,gyro(:,3),'r')
    %axis([time(1) time(end) -1.5 1.5])
    legend('x','y','z')
    title('Gyroscope')
end

if plotNormBias == 1
    normBias = zeros(size(time,1),1);
    for i = 1:size(time,1)
        normBias(i) = norm(gyro(i,:));
    end
    figure
    plot(time,normBias)
    title('Norm Bias')
end

if plotBias == 1
    figure
    hold on
    plot(time,bias(:,1))
    plot(time,bias(:,2),'g')
    plot(time,bias(:,3),'r')
    %axis([time(1) time(end) -1.5 1.5])
    legend('x','y','z')
    title('Bias')
end

if plotQuat == 1
    figure
    title('quaternion interpolated')
    for i = 1:4
        subplot(2,2,i)
        hold on
        plot(time,qNoise(:,i))
        plot(time,q(:,i),'r')
        xlabel('Time (s)')
        hold off
        legend('Noisy','Pure')
    end
end

if plotEuler == 1
    figure
    hold on
    plot(time,euler(:,1))
    plot(time,euler(:,2),'r')
    plot(time,euler(:,3),'g')
    legend('roll','pitch','yaw')
    title('Euler angles')
end


%% EKF
gyroEKF = gyro;
biasEKF = bias;
accEKF = acc;
magEKF = mag;
qEKF = q;

P = eye(7)*10;

sigma_r_sqr = 0.001^2; % 0.01
sigma_w_sqr = 0.001^2; % 0.01

R_acc = eye(3) * 0.05;%0.005;
R_mag = eye(3) * 0.2;%0.005;

x_state_pred = zeros(7,size(time,1));
x_state_corr = zeros(7,size(time,1));
x_state_pred(:,1) = [0 0 0 1 0 0 0]';%[biasEKF(1,:)';q(1,:)'];
x_state_corr(:,1) = [0 0 0 1 0 0 0]';%[biasEKF(1,:)';q(1,:)'];

onlyPrediction = 1;
predictAcc = 2;
predictMag = 3;
predictAll = 4;

ekf_type = 4;

type = 2;

for t = 2:size(time,1)
    
    dt = time(t)-time(t-1);
    
    switch ekf_type
        case onlyPrediction
            [x_state_pred(:,t),P] = ekf_predict(x_state_corr(:,t-1),P,dt,gyroEKF(t,:),sigma_w_sqr,sigma_r_sqr);
            x_state_corr(:,t) = x_state_pred(:,t);
        case predictAcc
            [x_state_pred(:,t),P] = ekf_predict(x_state_corr(:,t-1),P,dt,gyroEKF(t,:),sigma_w_sqr,sigma_r_sqr);
            [x_state_corr(:,t),P] = ekf_update_acc(x_state_pred(:,t),P,R_acc,acc(t,:)',type);
        case predictMag
            [x_state_pred(:,t),P] = ekf_predict(x_state_corr(:,t-1),P,dt,gyroEKF(t,:),sigma_w_sqr,sigma_r_sqr);
            [x_state_corr(:,t),P] = ekf_update_mag(x_state_pred(:,t),P,R_mag,mag(t,:)',type);
        case predictAll
            [x_state_pred(:,t),P] = ekf_predict(x_state_corr(:,t-1),P,dt,gyroEKF(t,:),sigma_w_sqr,sigma_r_sqr);
            [x_state_corr(:,t),P] = ekf_update_acc(x_state_pred(:,t),P,R_acc,accEKF(t,:)',type);
            [x_state_corr(:,t),P] = ekf_update_mag(x_state_corr(:,t),P,R_mag,magEKF(t,:)',type);
    end
    
    %x_state_pred(:,t) = x_state_corr(:,t);
    
end

if plotQuatFinal == 1
    figure
    for i = 1:4
        subplot(2,2,i)
        hold on
        plot(time,x_state_corr(i+3,:),'r')
        plot(time,qEKF(:,i))
        xlabel('Time (s)')
        axis([time(1) time(end) -1 1])
        legend('EKF','Real')
        hold off
    end
end
if plotBiasFinal == 1
    figure
    for i = 1:3
        subplot(3,1,i)
        hold on
        plot(time,x_state_corr(i,:),'r')
        plot(time,bias(:,i))
        xlabel('Time (s)')
        legend('EKF','real')
        hold off
    end
end

if plotPredictCorr == 1
    figure
    for i = 1:4
        subplot(2,2,i)
        hold on
        plot(time,x_state_pred(i+3,:))
        plot(time,x_state_corr(i+3,:),'r')
        legend('pred','corr')
    end
end

if plotEulerEKF == 1
    eulerEKF = zeros(size(time,1),3);
    for t = 1:size(time,1)
        [eulerEKF(t,1), eulerEKF(t,2), eulerEKF(t,3)] = quat2euler(x_state_corr(4:7,t));
    end
    figure
    for i = 1:3
        subplot(3,1,i)
        hold on
        plot(time,euler(:,i))
        plot(time,eulerEKF(:,i),'r')
        switch i
            case 1
                ylabel('Roll')
            case 2
                ylabel('Pitch')
            case 3
                ylabel('Yaw')
        end
        legend('Inter','EKF')
    end
end

%[roll,pitch,yaw] = quat2euler(x_state_corr(4:7,end));
%display(['Roll:' num2str(rad2deg(roll))])
%display(['Pitch:' num2str(rad2deg(pitch))])
%display(['Yaw:' num2str(rad2deg(yaw))])