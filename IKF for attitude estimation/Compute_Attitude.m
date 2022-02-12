%-------- Souhaiel Ben Salem --------------



% -------------------------------------------------------------------
% Attitude estimation using a quaternion-based indirect Kalman filter
% -------------------------------------------------------------------

function [q4, eulercom4, bahat, bghat] = Compute_Attitude(yg,ya,ym,tt,Rg,Ra,Rm)

D2R = pi/180;                               % Conversion from deg to rad
load('IMU_Data.mat');

% Number of data points : N
N = max(size(ya));

% creating ̃g and ̃m with α= 50

alpha = 50*D2R;
g_tilde = [0;0;-9.81];
m_tilde = [cos(alpha);0;-sin(alpha)];


% creating the bias covariance matrices Q_bg,Q_ba and Q

Qb_g=0.000001*eye(3) ; 
Qb_a=0.000001*eye(3) ; 
Q=blkdiag(0.25*Rg ,Qb_g, Qb_a);
size(Q) ;

% --------------------------------------------------------
% Kalman Filter
% --------------------------------------------------------

% q4: quaternion

q4 = zeros(4,N);

% eulercom4: euler angles
eulercom4 = zeros(3,N);

% Estimated bias for gyroscope (bghat) and accelerometer (bahat) 
bghat = zeros(3,1);
bahat = zeros(3,1);

% inital orientation estimation using the TRIAD method
yabar = ya(:,1) / norm(ya(:,1));
ymbar = ym(:,1) / norm(ym(:,1));

foo1 = cross(yabar,ymbar) / norm( cross(yabar,ymbar) );
C = [ -cross(yabar,foo1)  , foo1 , yabar ] ;
q4(:,1) = dcm2quaternion(C);

% Kalman filter state
x = zeros(9,1);

% creating an initial state error covariancematrix P=diag[1*I,0.000001*01*I] :

P=blkdiag( 0.01*eye(3) , 0.000001*eye(3), 0.000001*eye(3)) ;

% creating an initial Ω(from eq.(18) in thepaper) with first angular
% velocity measurements in the file “IMUData.mat” :

% yg is the angular velocity : the angular velocity is measured by the
% gyroscope
Omega=[0 -yg(1,1) -yg(2,1) -yg(3,1) ; 
       yg(1,1) 0 yg(3,1) -yg(2,1) ;
       yg(2,1) -yg(3,1) 0 yg(1,1); 
       yg(3,1) yg(2,1) -yg(1,1) 0] ;
   
% variable used in the adaptive algorithm      
r2count = 100;

% parameter for adaptive algorithm
M1 = 3;
M2 = 3;
gamma = 0.1;

R = zeros(3,3*N);

% Kalman filter loop
for i = 2:N
    % creating the sampling period T :
    
    % sampling period
    T=tt(i)-tt(i-1);
    
    % Creating the matrix A  (from eq.  (9)) in the paper : 
     A = [ -vec2product(yg(:,i)-ba) -0.5*eye(3) zeros(3);zeros(6,9) ] ;
     
    
    % Implementing the approximation of Phi_k (from eq.  (15) in the
    % paper after discretization):
    
    Phi=eye(9,9) + A*T+0.5*A^2*T^2 ; 
    
    % In the paper we used an approximated version ofQdin order to reduce the compu-tational cost. 
    % Thie approximation is obtained as follows:
    % Discritized Q_d,k
    Qd=Q*T+0.5*A*Q+0.5*Q*A.' ;
    x = Phi * x;
    
    % creating P− (from eq.  (17) in the paper) : 
    
    % Computing P_k+1 
    P_ = Phi*P*Phi.' + Qd;

   

    
    % We recreate Ω and associate to it next angular velocity measurements values:
    
       Omega=[0 -yg(1,i) -yg(2,i) -yg(3,i) ; 
       yg(1,i) 0 yg(3,i) -yg(2,i) ;
       yg(2,i) -yg(3,i) 0 yg(1,i); 
       yg(3,i) yg(2,i) -yg(1,i) 0] ; 
       
  
    % creating a quaternion (from eq.  (18) inthe paper):
    
    %Computing the quaternion
    Omega_prev = [0, -yg(1,i-1), -yg(2,i-1), -yg(3,i-1);
                  yg(1,i-1), 0, yg(3,i-1), -yg(2,i-1);
                  yg(2,i-1), -yg(3,i-1), 0, yg(1,i-1);
                  yg(3,i-1), yg(2,i-1), -yg(1,i-1), 0];
              
     q4(:,i) = (eye(4)+0.75*Omega*T - 0.25*Omega_prev*T - norm(yg(:,i),2)^2*T^2*eye(4)/6 - 1/24*Omega*Omega_prev*T^2 - 1/48 * norm(yg(:,i),2)^2 * Omega*T^3)*q4(:,i-1);
     Cq = quaternion2dcm(q4(:,i));
    % ----------------------------------------------------
    % two step measurement update
    % ----------------------------------------------------
  
    
    % implementing Z_a,k and H_a,k from eq.19 in the paper
    Ha = [2*Cq*vec2product(g_tilde), zeros(3), eye(3)];
    za = ya(:,i)-Cq*g_tilde;
    H1 = Ha;
    z1 = za;
    
    
    % adaptive algorithm
    fooR1 = (z1 - H1*x) * (z1 - H1*x)';
    R(:,3*(i-1)+1:3*i) = fooR1;
    uk = fooR1;
    for j = i-1:min([i-(M1-1),1])
        uk = uk + R(:,3*(j-1)+1:3*j);
    end
    uk = uk / M1;
  
    
    fooR2 = H1*P*H1' + Ra;
  
    [u,s,v] = svd(uk);
    u1 = u(:,1);
    u2 = u(:,2);
    u3 = u(:,3);
    
    lambda = [ s(1) , s(2) , s(3)];
    mu =  [ u1' * fooR2 * u1 , u2' * fooR2 * u2 , u3' * fooR2 * u3];
    
    if ( max(lambda - mu) > gamma )
      r2count = 0;
      Qstar = max(lambda(1)-mu(1),0)*u1*u1' + max(lambda(2) -mu(2),0)*u2*u2' + max(lambda(3)-mu(3),0)*u3*u3';
    else
      r2count = r2count + 1;
      if ( r2count < M2 )
        Qstar = max(lambda(1)-mu(1),0)*u1*u1' + max(lambda(2) -mu(2),0)*u2*u2' + max(lambda(3)-mu(3),0)*u3*u3';
      else
        Qstar = zeros(3,3);
      end
    end
    
    
    
    % Computing K_a,k (eq.19 in the paper)
    Ka = P_*Ha'/(Ha*P_*Ha' + Ra + Qstar);
    
    
    % Computing Xhat_a,k (eq.19 in the paper)   
    xahat = x + Ka*(za - Ha*x);
    
    
    % Computing P_a,k (eq.19 in the paper)
    Pa = (eye(9) - Ka*Ha)*P_*(eye(9) - Ka*Ha)' + Ka*(Ra + Qstar)*Ka';

    
    
    % implementing the equation 20 in the paper
    qe = [1;xahat(1:3)];
    q4(:,i) = quaternionmul(q4(:,i),qe)/norm(q4(:,i),2);
    Cq = quaternion2dcm(q4(:,i));
    xahat(1:3) = 0;
    
    % Computing H_m,k  and Z_m,k (eq.22 in the paper)
    Hm = [2*Cq*vec2product(m_tilde), zeros(3), zeros(3)];
    zm = ym(:,i)-Cq*m_tilde;
    
    
    % eqs 21 Implementation
    Pm_ = [Pa(1:3,1:3) zeros(3,6);
            zeros(6,3) zeros(6,6)];
    r3 = Cq*[0;0;1];
    Km = [r3*r3' zeros(3,6);
          zeros(6,3) zeros(6,6)]*Pm_*Hm'/(Hm*Pm_*Hm'+Rm);
    x = xahat + Km*(zm-Hm*xahat);
    P = (eye(9)-Km*Hm)*Pa*(eye(9) - Km*Hm)' + Km*Rm*Km';
    
    bghat = bghat + x(4:6);
    x(4:6) = zeros(3,1);
    
    bahat = bahat + x(7:9);
    x(7:9) = zeros(3,1);
    
    qe = [1;xahat(1:3)];
    q4(:,i) = quaternionmul(q4(:,i),qe)/norm(q4(:,i),2);
    Cq = quaternion2dcm(q4(:,i));
    x(1:3) = 0;
    
    % converting the estimated quaternion to Euler angles
    eulercom4(:,i) = quaternion2euler(q4(:,i))/D2R;
    
end
