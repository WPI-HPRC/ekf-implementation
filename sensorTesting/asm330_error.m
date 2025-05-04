%% WPI HPRC - ASM330 Error Calibration
% Author: Daniel Pearson
% Version: 5/4/2025

clear variables; close all; clc;

%% === Load Data ===
rawASM = readtable('sitting_still.csv');

% Timestamp
time = rawASM.timestamp / 1000;

% Acceleration [g]
accelX = rawASM.ASMaccelX;
accelY = rawASM.ASMaccelY;
accelZ = rawASM.ASMaccelZ;

% Gyroscope [deg/s]
gyroX  = rawASM.ASMgyrX;
gyroY  = rawASM.ASMgyrY;
gyroZ  = rawASM.ASMgyrZ;

%% === Plot Raw Acceleration ===
figure('Name', 'ASM330 Acceleration Readings');
plot(time, accelX, 'DisplayName', 'Accel X'); hold on;
plot(time, accelY, 'DisplayName', 'Accel Y');
plot(time, accelZ, 'DisplayName', 'Accel Z'); hold off;
grid on; legend();
title('ASM330 Acceleration Readings');
xlabel('Time (s)');
ylabel('Acceleration (g)');

%% === Plot Raw Gyro ===
figure('Name', 'ASM330 Gyroscope Readings');
plot(time, gyroX, 'DisplayName', 'Gyro X'); hold on;
plot(time, gyroY, 'DisplayName', 'Gyro Y');
plot(time, gyroZ, 'DisplayName', 'Gyro Z'); hold off;
grid on; legend();
title('ASM330 Gyroscope Readings');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');

%% === Sampling Info ===
dt = mean(diff(time));
fs = 1 / dt;

%% === Accelerometer Error ===
RMS_accX = std(accelX);
RMS_accY = std(accelY);
RMS_accZ = std(accelZ);

VRW_accX = RMS_accX / sqrt(fs);
VRW_accY = RMS_accY / sqrt(fs);
VRW_accZ = RMS_accZ / sqrt(fs);

dev_accX = accelX - mean(accelX);
dev_accY = accelY - mean(accelY);
dev_accZ = accelZ - mean(accelZ);

pd_accX = fitdist(dev_accX, 'Normal');
pd_accY = fitdist(dev_accY, 'Normal');
pd_accZ = fitdist(dev_accZ, 'Normal');

distRange_acc = linspace(min([dev_accX; dev_accY; dev_accZ]), max([dev_accX; dev_accY; dev_accZ]), 100);

pdf_accX = pdf(pd_accX, distRange_acc);
pdf_accY = pdf(pd_accY, distRange_acc);
pdf_accZ = pdf(pd_accZ, distRange_acc);

% Plot Accelerometer Error Distributions
figure('Name', 'ASM330 Accelerometer Error Distribution');
subplot(3,1,1);
histogram(dev_accX, 'Normalization', 'pdf', 'FaceAlpha', 0.5); hold on;
plot(distRange_acc, pdf_accX, 'r', 'LineWidth', 2); grid on;
title('Accel X Error'); xlabel('Error (g)'); ylabel('PDF');

subplot(3,1,2);
histogram(dev_accY, 'Normalization', 'pdf', 'FaceAlpha', 0.5); hold on;
plot(distRange_acc, pdf_accY, 'g', 'LineWidth', 2); grid on;
title('Accel Y Error'); xlabel('Error (g)'); ylabel('PDF');

subplot(3,1,3);
histogram(dev_accZ, 'Normalization', 'pdf', 'FaceAlpha', 0.5); hold on;
plot(distRange_acc, pdf_accZ, 'b', 'LineWidth', 2); grid on;
title('Accel Z Error'); xlabel('Error (g)'); ylabel('PDF');

%% === Gyroscope Error ===
RMS_gyroX = std(gyroX);
RMS_gyroY = std(gyroY);
RMS_gyroZ = std(gyroZ);

VRW_gyroX = RMS_gyroX / sqrt(fs);
VRW_gyroY = RMS_gyroY / sqrt(fs);
VRW_gyroZ = RMS_gyroZ / sqrt(fs);

dev_gyroX = gyroX - mean(gyroX);
dev_gyroY = gyroY - mean(gyroY);
dev_gyroZ = gyroZ - mean(gyroZ);

pd_gyroX = fitdist(dev_gyroX, 'Normal');
pd_gyroY = fitdist(dev_gyroY, 'Normal');
pd_gyroZ = fitdist(dev_gyroZ, 'Normal');

distRange_gyro = linspace(min([dev_gyroX; dev_gyroY; dev_gyroZ]), max([dev_gyroX; dev_gyroY; dev_gyroZ]), 100);

pdf_gyroX = pdf(pd_gyroX, distRange_gyro);
pdf_gyroY = pdf(pd_gyroY, distRange_gyro);
pdf_gyroZ = pdf(pd_gyroZ, distRange_gyro);

% Plot Gyroscope Error Distributions
figure('Name', 'ASM330 Gyroscope Error Distribution');
subplot(3,1,1);
histogram(dev_gyroX, 'Normalization', 'pdf', 'FaceAlpha', 0.5); hold on;
plot(distRange_gyro, pdf_gyroX, 'r', 'LineWidth', 2); grid on;
title('Gyro X Error'); xlabel('Error (deg/s)'); ylabel('PDF');

subplot(3,1,2);
histogram(dev_gyroY, 'Normalization', 'pdf', 'FaceAlpha', 0.5); hold on;
plot(distRange_gyro, pdf_gyroY, 'g', 'LineWidth', 2); grid on;
title('Gyro Y Error'); xlabel('Error (deg/s)'); ylabel('PDF');

subplot(3,1,3);
histogram(dev_gyroZ, 'Normalization', 'pdf', 'FaceAlpha', 0.5); hold on;
plot(distRange_gyro, pdf_gyroZ, 'b', 'LineWidth', 2); grid on;
title('Gyro Z Error'); xlabel('Error (deg/s)'); ylabel('PDF');

%% === Summary Table ===
fprintf('\n=== ASM330 SENSOR ERROR SUMMARY ===\n');
fprintf('Accelerometer RMS [g]           : X = %.4f, Y = %.4f, Z = %.4f\n', RMS_accX, RMS_accY, RMS_accZ);
fprintf('Accelerometer VRW [g/sqrt(Hz)]  : X = %.4f, Y = %.4f, Z = %.4f\n', VRW_accX, VRW_accY, VRW_accZ);

fprintf('Gyroscope RMS [deg/s]           : X = %.4f, Y = %.4f, Z = %.4f\n', RMS_gyroX, RMS_gyroY, RMS_gyroZ);
fprintf('Gyroscope VRW [deg/sqrt(Hz)]    : X = %.4f, Y = %.4f, Z = %.4f\n', VRW_gyroX, VRW_gyroY, VRW_gyroZ);
