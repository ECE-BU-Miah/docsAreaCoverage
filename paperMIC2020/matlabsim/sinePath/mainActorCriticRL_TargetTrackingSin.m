% Odometric localization or dead reckoning%   Script for simulating mobile robot.
%Edited by AMR Elhussein on May, 29,2020.
%gradient descent Appraoch, three cases comined fixed point, horzintal and inclined line, 
%converged Example
function mainActorCriticRL_TargetTrackingSin

% Created by S. Miah on Nov. 03, 2015.
%edited by AMR Elhussein on Aug, 30,2019.
close all
clear all
clc

disp('Please wait while the simulation runs ...');

% ---------------------
% SIMULATION PARAMETERS
% ---------------------

% Simulation time
t0 = 0; tf = 60; % initial and final simulation time [s]
T = 0.0009;  % Sampling time [s]
tsteps = ceil((tf-t0)/T); % number of time steps
dt = T*(0:tsteps)'; % Discrete time vector (include time t0 to plot initial state too)

d = 0.5; %[m] safe distance
%ROBOT PARAMETERS
%==========================================================================
l = 0.381; % [m]
rad = 0.0925; % [m] radius
%==========================================================================


% dimension of system;
N = 2;  % state vector dimension
M = 2;  % input vector dimension


% Leader's initial position
pLeaderInit = [0;0];%+ rand(2,1); %[m]
pLeaderTheta(:,1)= 0:(4*pi)/(tsteps-1):4*pi; %[rad]
alpha = 4*pi/tf;
pLeaderTheta(tsteps+1,:)=4*pi;

QD(1,:) = [pLeaderInit' cos(0.1*pLeaderTheta(1,:))];


QDinit = QD(1,:);
%Leader's velocity 
vLeader = 2; % [m/s]


% Follower robot initialization% 
qp = [-2;-2]; % [m] Follower robot's position
qTheta = 0 ;% [rad] Follower robot's orientation 
 qInit = [qp',qTheta] ;%+ rand(1,3); % row vector
vMax = 5; % [m/s] Follower's maximum linear speed 

Q(1,:) = qInit;

% ----------------------
% RUN SIMULATION 
% ----------------------

% Run the true model
q(1,:) = qInit';
pk = pLeaderInit;

%initiating matrices
P = [3  0   1   0.5;
      0   3   0.5    1;
      2 0.5   3      0;
    0.5   2    0      3]
 P = P+5*rand;
eig(P)
Pinit = P;

%Q and R matrices
Q = 0.001*[1  0;
                 0  1];
R = 0.00000001*[1 0;
                 0 1];
Pbar = blkdiag(Q,R);

%Learning parameters
lc = 0.00001; %learnging rate
la= 0.005; %actor learning rate

%initlize actor weights
Wa=0.001*rand(2,2);

%reconstruct the P matrix to get the wheights matrix
w(1,1) = 0.5 * P(1,1);
w(2:4,1)=P(1,2:4);
w(5,1) = 0.5*P(2,2);
w(6:7,1) =P(2,3:4) ;
w(8,1) = 0.5*P(3,3);
w(9,1) =P(3,4) ;
w(10,1) = 0.5*P(4,4);

       
% Store weight vector over time (every (n+m)*(n+m+1)/2 time steps).        
W = zeros((N+M)*(N+M+1)/2,floor(tsteps/((N+M)*(N+M+1)/2))+1);
WAA = zeros(4,floor(tsteps/((N+M)*(N+M+1)/2))+1);
W(:,1) = w;
WAA(:,1)=Wa(:);

%intilize tracking error and policy
thetaPrime(1,1) = atan2(QD(1,2)-q(1,2),QD(1,1)-q(1,1)); %
ex =QD(1,1)-q(1,1)-d*cos(thetaPrime(1,1));
ey=QD(1,2)-q(1,2)-d*sin(thetaPrime(1,1));
trackingError(1,:) = [sqrt(ex^2+ey^2), pi2pi(thetaPrime(1,:)-q(1,3))]  ; 
u(1,:) = Wa*trackingError(1,:)';%finding the policy;   



%limit control inputs 
u(1,1) = sign(u(1,1))*min(vMax,abs(u(1,1)));%limiting the linear speed 
if u(1,2)<80*pi/180 | u(1,2)<-80*pi/180 
    u(1,2) = sign(u(1,2))*min(80*(pi/180),abs(u(1,2)));%limiting the steering angle
end

uk = u(1,:)';
z(1,:) = [trackingError(1,:) u(1,:)];    

%calculte the value function
valueFunction(1,:) = 0.5*( trackingError(1,:)*Q*trackingError(1,:)'+ u(1,:)*R*u(1,:)');

%intilize neurons activation functions
m=1; n=1;c=0;
for m = 1:4
    for n = m:4
        c=c+1;
        rho(1,c)= z(1,m)*z(1,n);
    end
end
    

kappa = 1;  % data collection index 
indexWeight= 1; %weight saving index
for k = 1:tsteps    % Main timing loop, k = time index
    
%generating sine path for the leader
%     pKPlus1 =  [(pLeaderTheta(k+1,:));vLeader*sin(0.5*pLeaderTheta(k+1,:))];
%     QD(k+1,:) = [pKPlus1' cos(0.5*pLeaderTheta(k+1,:))];    
%     pk = pKPlus1;
    
    pKPlus1 =  [alpha*k*T;vLeader*sin(0.5*alpha*k*T)];
    QD(k+1,:) = [pKPlus1' cos(0.5*alpha*k*T)];    
    pk = pKPlus1;

    % update follower's position and orientation;    
    q(k+1,:) = DDMR_modelEuler(uk,qInit',T);
    qInit= q(k+1,:);
    
    %tracking error calculations
    thetaPrime(k+1,:) = atan2(QD(k+1,2)-q(k+1,2),QD(k+1,1)-q(k+1,1)); 
    ex = QD(k+1,1)-q(k+1,1)-d*cos(thetaPrime(k+1,1));
    ey = QD(k+1,2)-q(k+1,2)-d*sin(thetaPrime(k+1,1));
    trackingError(k+1,:) = [sqrt(ex^2+ey^2), pi2pi(thetaPrime(k+1,:)-q(k+1,3))] ;
    
    %finding the policy
    u(k+1,:) = Wa*trackingError(k+1,:)';
    %limit control inputs 
    u(k+1,1) = sign(u(k+1,1))*min(vMax,abs(u(k+1,1)));%limiting the linear speed 
    u(k+1,2) = sign(u(k+1,2))*min(80*(pi/180),abs(u(k+1,2)));%limiting the steering angle
    uk = u(k+1,:)';

    %combining the tracking error and the control policy in one vector
    z(k+1,:) = [trackingError(k+1,:)  u(k+1,:)];

    %neuron activation functions
    c = 0;
   for i =1:4
       for j=i:4
        c=c+1;
        rho(k+1,c) = z(k+1,i)*z(k+1,j);  % neuron activation functions 
       end
   end
   
   %recording the equation
   rhoTelda(kappa,:) = rho(k,:)-rho(k+1,:); %Need for Lambda matrix  
   valueFnDesired(kappa,1) = z(k,:)*Pbar*z(k,:)';%the v vector  
   
   if mod(k,10)==0
        PWc = w; %saving previous critic weights
        kappa = 1;
        Pprevious = P;
        w = PWc - lc*rhoTelda'*(rhoTelda*PWc-valueFnDesired); % critic wheight matrix
        W(:,indexWeight+1) = w;
        WAA(:,indexWeight+1)=Wa(:);
        indexWeight = indexWeight + 1;
        %current wc
        CWc = w; 
        err = norm (CWc - PWc);
        if err < 0.0001
            disp('Congratz converging whgihts')
        end

        % Reconstruct  the P matrix
        P(1,1)= 2 *w(1,1);
        P(1,2:4)=w(2:4,1);
        P(2,2)=2*w(5,1);
        P(2,3:4)=w(6:7,1) ;
        P(3,3) = 2*w(8,1);
        P(3,4)=w(9,1) ;
        P(4,4)=2*w(10,1);

        for m = 1:4
            for n = 1:4
                P(n,m) = P(m,n);  
            end
        end


        %update policy
        uk =  1*inv(P(3:4,3:4))*P(3:4,1:2)*trackingError(k+1,:)';

        %updating actor weights
        Waa=(Wa*trackingError(k+1,:)')-(uk+randn(2,1));
        Wa=Wa-la*Waa*trackingError(k+1,:);

        %limit control inputs 
        uk(1) = sign(uk(1))*min(vMax,abs(uk(1)));%limiting the linear speed 
        uk(2) = sign(uk(2))*min(80*(pi/180),abs(uk(2)));%limiting the steering angle        
        z(k+1,:) = [trackingError(k+1,:)  uk'];

        %tracking the eigen values
         eig(P)
         %update the neuron activation function for the last time step 
         c = 0;
         for i =1:4
            for j=i:4
                c=c+1;
                rho(k+1,c) = z(k+1,i)*z(k+1,j);  % neuron activation functions 
            end
         end  
        else
            k=k+1;
            kappa = kappa + 1; % Data collection index
    end
    
end  % End of timing loop

if mod(k,10)==0
    k=k+1;
end
 
Qf_DT = q; % include intial state q0 too

% Determine the robot's actualpositions (see class note)
followerPose = Qf_DT(1:k,:);
leaderPose = QD;
poseError = leaderPose - followerPose; % error with respect to global coordinate systems

% Plot weights 
figure
plot((0:floor(tsteps/((N+M)*(N+M+1)/2)))*(N+M)*(N+M+1)/2*T,W,'Linewidth',1.5);
set(gca, 'PlotBoxAspectRatio',[3 1 1]);
xlabel('Time [s]');
ylabel('{\bf \omega_c}');
grid on
savefilename = 'OUT/weightSineWave';
saveas(gcf, savefilename, 'fig');
print('-depsc2', '-r300', [savefilename, '.eps']);

%Plot Actor Weights
figure
plot((0:floor(tsteps/((N+M)*(N+M+1)/2)))*(N+M)*(N+M+1)/2*T,WAA,'Linewidth',1.5);
xlabel('Time [s]');
ylabel('{\bf W_a}');
grid on
set(gca, 'PlotBoxAspectRatio',[3 1 1]);
savefilename = 'OUT/weightActorSineWave';
saveas(gcf, savefilename, 'fig');
print('-depsc2', '-r300', [savefilename, '.eps']);

% Plot system's performance (trajectory, state error, control inputs)

generatePlots(length(dt), dt(1:k,:), followerPose(1:k,:) , leaderPose(1:k,:), poseError(1:k,:), u(1:k,:),trackingError(1:k,:));
disp('... done.');



% Numerical integration of robot's kinematic model
function [ x ] = DDMR_modelEuler(u, xk, T)

x_mem = xk;
% ------------------------
% Compute the model output
% ------------------------

% Euler (or rectangular) integration
x_mem = x_mem + [u(1)*T*cos(x_mem(3)+u(2)); ...
                 u(1)*T*sin(x_mem(3)+u(2)); ...
                 T*u(1)*sin(u(2))/0.15];
% x_mem = x_mem + [u(1)*T*cos(x_mem(3)); ...
%                  u(1)*T*sin(x_mem(3)); ...
%                  T*u(2)];

% Keep robot's orientation between -pi to pi
x_mem(3) = pi2pi(x_mem(3));
 
% if x_mem(3)>3.1416  % pi is aproximately 3.1416
%     x_mem(3) = -(2*pi - x_mem(3));
% end


% -----------------------
% Set the function output
% -----------------------

x = x_mem;

function angle = pi2pi(angle)

%function angle = pi_to_pi(angle)
% Input: array of angles.
% Tim Bailey 2000

angle = mod(angle, 2*pi);

i=find(angle>pi);
angle(i)=angle(i)-2*pi;

i=find(angle<-pi);
angle(i)=angle(i)+2*pi;

function generatePlots(Tn, t, actualStates, desiredStates, error, u,trackingError)    
    %close all; % close all opened figures
    % Tn = number of discrete time/path parameter points 
    % t = time/path parameter history, dimension = Tn x 1 
    % actualState = actual state of the system, dimension = Tn x n
    % desiredState = desired state of the system, dimension = Tn x n
    % error = desired state - actual state, dimension =  Tn x n
    % u = control inputs, dimension = Tn x m         
 
    % Plot the robot's  velocities, 
    figure
    subplot(2,1,1)
    plot(t(1:end-1),u(1:end-1,1), 'k-','LineWidth', 1.5);   
    xlabel('Time [s]');
    ylabel('Linear speed [m/s]');        
    grid on
    set(gca, 'PlotBoxAspectRatio',[3 1 1]);
    subplot(2,1,2)
    plot(t(1:end-1),180/pi*u(1:end-1,2), 'k--','LineWidth', 1.5);   
    xlabel('Time [s]');
    ylabel('Steering angle [deg]');
    grid on
    set(gca, 'PlotBoxAspectRatio',[3 1 1]);
    savefilename = ['OUT/controlInputSineWave'];
    saveas(gcf, savefilename, 'fig');
    print('-depsc2', '-r300', [savefilename, '.eps']);
    
    % Create a movie of the simulation (path/trajectory)
    

    xmax = max(max(actualStates(:,1),desiredStates(:,1)));
    xmin = min(min(actualStates(:,1),desiredStates(:,1)));
    ymax = max(max(actualStates(:,2),desiredStates(:,2)));
    ymin = min(min(actualStates(:,2),desiredStates(:,2)));
          
%     vid = VideoWriter('OUT/trajectoryRectilinear.avi');
    vid.Quality = 100;
    vid.FrameRate = 5;
%     open(vid)
    
    fig=figure;
    clf reset;    
    
    for i = 1:100:Tn-1,
        clf;
        box on;
         axis([xmin-2 xmax+2 ymin-2 ymax+2]);
%         axis equal;
        axis manual;
        [Xa,Ya] = plot_DDMR(actualStates(i,:),axis(gca)); % 
        
        hold on;
        
        [Xl,Yl] = plot_DDMR(desiredStates(i,:),axis(gca));
        fill(Xl,Yl,'b'); 
        
        desired = plot(desiredStates(1:i,1),desiredStates(1:i,2),'LineWidth',1.5);
        
        
        hold on
        actual = plot(actualStates(1:i,1),actualStates(1:i,2),'k--');
        fill(Xa,Ya,'r');
        hold off;
        xlabel('x [m]');
        ylabel('y [m]');
%         F = getframe(fig);
%         writeVideo(vid,F);          
    end
    [Xa,Ya] = plot_DDMR(actualStates(1,:),axis(gca)); % DDMR => Differential drive mobile robot
    grid on
    set(gca, 'PlotBoxAspectRatio',[3 1 1]);
    hold on
    plot(Xa,Ya);    
    hold on
    legend([actual desired],'Robot', 'Target');
    set(gca, 'PlotBoxAspectRatio',[3 1 1]);
    savefilename = 'OUT/trajectorySineWave';
    saveas(gcf, savefilename, 'fig');
    print('-depsc2', '-r300', [savefilename, '.eps']);
       
%     close(vid);

    % Create the movie and save it as an AVI file
    % winopen('OUT/trajectory.avi')

    % plot error
    figure
    subplot(3,1,1)
    plot(t(1:end-1),error(1:end-1,1), 'k-','LineWidth', 1.5);   
    xlabel('Time [s]');
    ylabel('x_e');        
    grid on
    set(gca, 'PlotBoxAspectRatio',[3 1 1]);
    subplot(3,1,2)
    plot(t(1:end-1),error(1:end-1,2), 'k-','LineWidth', 1.5);   
    xlabel('Time [s]');
    ylabel('y_e [m]');
    grid on
    set(gca, 'PlotBoxAspectRatio',[3 1 1]);
    subplot(3,1,3)
    plot(t(1:end-1),error(1:end-1,3), 'k-','LineWidth', 1.5);   
    xlabel('Time [s]');
    ylabel('\theta_e [rad]');
    grid on
    set(gca, 'PlotBoxAspectRatio',[3 1 1]);
    %plot euclidean distance rho_e
   
    savefilename = ['OUT/stateErrorSineWave'];
    saveas(gcf, savefilename, 'fig');
    print('-depsc2', '-r300', [savefilename, '.eps']);
    
    
    figure
    plot(t(1:end-1),trackingError(1:end-1,1), 'k-','LineWidth', 1.5);   
    xlabel('Time [s]');
    ylabel('$$\tilde{\rho_k}$$','Interpreter','latex');        
    grid on
    set(gca, 'PlotBoxAspectRatio',[3 1 1]);
    savefilename = ['OUT/euclideanDistanceSineWave'];
    saveas(gcf, savefilename, 'fig');
    print('-depsc2', '-r300', [savefilename, '.eps']);
    

function [X,Y] = plot_DDMR(Q,AX)
% PLOT_UNICYCLE   Function that generates lines for plotting a unicycle.
%
%    PLOT_UNICYCLE(Q,AX) takes in the axis AX = axis(gca) and the unicycle
%    configuration Q and outputs lines (X,Y) for use, for example, as:
%       fill(X,Y,'b')
%    This must be done in the parent script file.

x     = Q(1);
y     = Q(2);
theta = Q(3);

l1 = 0.02*max([AX(2)-AX(1),AX(4)-AX(3)]);
X = [x,x+l1*cos(theta-2*pi/3),x+l1*cos(theta),x+l1*cos(theta+2*pi/3),x];
Y = [y,y+l1*sin(theta-2*pi/3),y+l1*sin(theta),y+l1*sin(theta+2*pi/3),y];



%generate semetric positive definite matrix  
function A = generateSPDmatrix(n)
k =2;
x =rand(n,1);
F = triu(bsxfun(@min,x,x'))*rand(n,1);
A = k*(diag(x)+F + F');
A = A + 0.5*(diag(randi(n,n,1)));

% for i = 1:5
%    for j = i:5
%         if i == j 
%             A(i,j) = - A(i,j);
%         end
%     end
% % end
% A = 0.5*rand(n,n)
% A = (A+A')
% A = A + 5*eye(n)
% % A = A + diag(sum(A,2)+ randi(5,5,1))
%inv(A(4:5,4:5))*A(4:5,1:3)
 eig(A)
 
 tt= 1;
        