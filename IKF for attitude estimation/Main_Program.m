% Implementation of the paper entitled: Orientation Estimation Using a Quaternion-Based Indirect Kalman Filter
% With Adaptive Estimation of External Acceleration

%-------- Souhaiel Ben Salem --------------


clear all
clc

%--------------------------------------------------------------------------------------------------
% Main program to read IMU data, run the fuction that estimates quaternion/Euler angles, and biases 
% and plots IMU data, external acceleration and estimated Euler angles
%--------------------------------------------------------------------------------------------------
R2D = 180/pi;                           % Conversion from rad to deg

% reading the IMU data from“IMUData.mat” using the “load” Matlab function. 
% and  defining and explaining the variables that are in this file (.mat)

load('IMU_Data.mat');
% display the meaning, size and type of each variable
fprintf('acc stands for acceleration variable and it is of size : %g x %g  double \n' , size(acc))
fprintf('ba stands for accelerometer bias  and it is of size : %g x %g  double \n' , size(ba))
fprintf('bg stands for gyroscope bias  and it is of size : %g x %g  double \n' , size(bg))
fprintf('euler stands for the euler angles measurements  and it is of size : %g x %g  double \n' , size(euler))
fprintf('N stands for the number of measurements and it is of size : %g int \n' , size(N))
fprintf('R2D stands for passage from radian to degree coefficient and it is of size : %g  double \n' , size(R2D))
fprintf('tt stands for measurement sampling time  and it is of size : %g x %g  double \n' , size(tt))
fprintf('wa stands for the  noise on the accelerometer measurements  and it is of size : %g x %g  double \n' , size(wa))
fprintf('wg stands for the noise on the gyroscope measurements and it is of size : %g x %g  double \n' , size(wg))
fprintf('wm stands for the noise on the magnetic sensor measurements  and it is of size : %g x %g  double \n' , size(wm))
fprintf('ya stands for the output of the accelerometer and it is of size : %g x %g  double \n' , size(ya))
fprintf('yg stands for the output of the gyroscope and it is of size : %g x %g  double \n' , size(yg))
fprintf('ym stands for the output of the magnetic sensor and it is of size : %g x %g  double \n' , size(ym))

%remainder of your code here
    
% plotting these variables

figure()
% plotting the acceleration
title('first figure')
subplot(3,1,1)
plot(acc(1,:))
title('acceleration on the x-axis (acc\_x)')
subplot(3,1,2) 
plot(acc(2,:))
title('acceleration on the y-axis (acc\_y)')
subplot(3,1,3)
plot(acc(3,:))
title('acceleration on the z-axis (acc\_z)')
figure()
% plotting the accelerometer noise
subplot(3,3,1)
plot(wa(1,:))
title('accelerometer noise on the x-axis (wa\_x)')

subplot(3,3,2)
plot(wa(2,:))
title('accelerometer noise on the y-axis (wa\_y)')

subplot(3,3,3)
plot(wa(3,:))
title('accelerometer noise on the z-axis (wa\_z)')

% plotting the gyroscope noise
subplot(3,3,4)
plot(wg(1,:))
title('gyroscope noise on the x-axis (wg\_x)')

subplot(3,3,5)
plot(wg(2,:))
title('gyroscope noise on the y-axis (wg\_y)')

subplot(3,3,6)
plot(wg(3,:))
title('gyroscope noise on the z-axis (wg\_z)')
% plotting the magnetic senosr noise

subplot(3,3,7)
plot(wm(1,:))
title('magnetic sensor noise on the x-axis (wm_x)')

subplot(3,3,8)
plot(wm(2,:))
title('magnetic sensor noise on the y-axis (wm_y)')

subplot(3,3,9)
plot(wm(3,:))
title('magnetic sensor noise on the z-axis (wm_z)')


% plotting the output of each sensor 
figure()

%  the output of the accelerometer ya
subplot(3,3,1)
plot(ya(1,:))
title('accelerometer: ya\_x')

subplot(3,3,2)
plot(ya(2,:))
title('accelerometer: ya\_y')

subplot(3,3,3)
plot(ya(3,:))
title('accelerometer: ya\_z')

% the output of the gyroscope yg
subplot(3,3,4)
plot(yg(1,:))
title('gyroscope: yg\_x')

subplot(3,3,5)
plot(yg(2,:))
title('gyroscope:yg\_y')

subplot(3,3,6)
plot(yg(3,:))
title('gyroscope: yg\_z')

% the output of the magnetic sensor ym
subplot(3,3,7)
plot(ym(1,:))
title('magnetic sensor: ym\_x')

subplot(3,3,8)
plot(ym(2,:))
title('magnetic sensor: ym\_y')

subplot(3,3,9)
plot(ym(3,:))
title('magnetic sensor: ym\_z')

% create the three noise error covariance matrices for gyroscope data “Rg”, accelerometer data “Ra”
% and magnetometer data “Rm”, as given in Section V of the paper.

Ra= 0.0056*eye(3) ; % acceleormeter's noise error covariance matrix 
Rg= 0.003*eye(3)  ;% gyroscope's noise error covariance matrix 
Rm=0.001*eye(3) ; % magnetic sensor's noise error covariance matrix 

% Calling the ”computeattitude” function

[q4, eulercom4, bahat, bghat] = Compute_Attitude(yg,ya,ym,tt,Rg,Ra,Rm);



% Disoplaying the final results i.e real values against estimated values: 
 
figure()
subplot(311)
plot(tt,eulercom4(1,:))
hold on 
plot(tt,euler(1,:))
title('pitch')
legend('true pitch','estimated pitch')

subplot(312)
plot(tt,eulercom4(2,:))
hold on
plot(tt,euler(2,:))
title('roll')
legend('true roll','estimated roll')


subplot(313)
plot(tt,eulercom4(3,:))
hold on
plot(tt,euler(3,:))
title('yaw')
legend('true yaw','estimated yaw')



