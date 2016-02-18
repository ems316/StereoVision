function hwk4(invMat,dispx,lidar)

% Close any/all open figures, and open a new one associated with kbhit()
close all;

%[invMat,dispx,lidar] = calibrate();
%waitforbuttonpress;

 %now put reflector back on robot
% Do not change these variables
% global kbhit;
% kbhit = false;
dt= 0.1;
v = 0.25;
omega = 0;

    javaaddpath('./lcm.jar');
    javaaddpath('./vader_types.jar');
    % start the communication
    lc = lcm.lcm.LCM('udpm://239.255.76.67:7667?ttl=1');
    agg = lcm.lcm.MessageAggregator();
    agg.setMaxMessages(2);
    lc.subscribe('sickData', agg);
   

 x = [-1.25;-1;pi/2]; %new
 robot = iRobotCreate(1/dt, 5);



% Initialize the position and orientation covariance uncertainty.
P = [0.01 0 0; 0 0.01 0; 0 0 pi/2]; 

% This is to register kbhit functionality with the figure
iptaddcallback(gcf, 'KeyPressFcn', @my_kbhit);

% Plot the position of the LIDAR.  Do not change this.


% *** ADD NECESSARY CODE HERE ***
Q = [ .015, 0; 0, pi/150];
R = [.01, 0; 0, pi/180];
omega_correction = 0;
v_correction = 0;
state = 1;
last_error = 0;
accumulated_error = 0;
last_omega_corrupted= 0;
omega_corrupted =0;
error = 0;
change_in_error = 0;
last_errors = [0,0,0];
toc_flag = 1;
tic
while 1
    
    %%%%State%%%%
    %state 1: pt1 to pt2
    %state 2: pt2 to pt3
    %state 2.5: turn 90 degrees
    %state 3: pt3 to pt6
    %state 3.5: turn 90 degrees
    %state 4: pt6 to pt1
    %state 5: pt1 to pt2
    %state 5.5: turn 90 degrees
    %state 6: pt2 to uppder center point
    %state 6.5: turn 90 degrees
    %state 7: upper center point to pt7
    %state 8: flag capture mode
     
   kp =25;
    kd = 15; %susceptible to noise
    ki = 0; %gets smoothing
    error = 0;
    pose = x;
    speed_straight = .4;  %.4 good
    v_turn = .35;
    [state,accumulated_error] = update_state_modded(state, pose,accumulated_error);
    zero_turn_omega = pi/2.5;  %2.5 googd
   if(state == 1)
        v = speed_straight;
        omega = 0;
      error =  -abs(0-(-1)*x(1)-(0-(-1.25))*x(2)+((0)*(-1))-(0*(-1.25)))/sqrt((0-(-1))^2+(0-(-1.25))^2);
       elseif (state == 1.5)
        omega = zero_turn_omega;
        v =0;
        error =0;  
        last_errors(1) = 0;
        last_errors(2) = 0;
        last_errors(3) = 0;
    elseif (state == 2)
      %kp = .7;
        %ki = .18;
        error = -1*pose(1)-0;
        v=speed_straight;
        omega = 0;
        last_errors(1) = 0;
        last_errors(2) = 0;
        last_errors(3) = 0;
    elseif (state == 3)
        %kp = .7;
        %ki = .18;
        error = pose(1)-0;
        v=-1*speed_straight;
        omega = 0;
        last_errors(1) = 0;
        last_errors(2) = 0;
        last_errors(3) = 0;
    elseif (state == 3.5)
        omega = zero_turn_omega;
        v =0;
        error =0;
        last_errors(1) = 0;
        last_errors(2) = 0;
        last_errors(3) = 0;
    elseif (state == 4)
%         circle_center = [0,-1];
%         radius = 1.25;
%         %v=rw
%         v = v_turn;
%         omega = -v_turn/radius;
%         error = sqrt((pose(1)-circle_center(1))^2 + (pose(2)-circle_center(2))^2) - radius;
%         line_error = -1*(pose(1)-(-1.25));
%         if line_error < error 
%            error = line_error; 
%         end
        error = -1*pose(1)-0;
        v=speed_straight;
        omega = 0;
        last_errors(1) = 0;
        last_errors(2) = 0;
        last_errors(3) = 0;
    elseif (state == 4.5)
        omega = zero_turn_omega;
        v =0;
        error =0;
        last_errors(1) = 0;
        last_errors(2) = 0;
        last_errors(3) = 0;
    elseif (state==5)
        error =  abs(0-(-1)*x(1)-((-.5-0)*x(2))+((-.5)*(-1))-(0*(0)))/sqrt((0-(-1))^2+(-.5-0)^2);
        v=speed_straight;
        omega = 0;
        last_errors(1) = 0;
        last_errors(2) = 0;
        last_errors(3) = 0;
        pose(1)
    elseif (state == 5.5)
        omega = -zero_turn_omega;
        v =0;
        error =0;
        last_errors(1) = 0;
        last_errors(2) = 0;
        last_errors(3) = 0;
    elseif (state == 6)
        pose(1)
        v = speed_straight;
        omega = 0;
        error = -1*(pose(1)+1);
        last_errors(1) = 0;
        last_errors(2) = 0;
        last_errors(3) = 0;
    elseif (state == 6.5) 
        omega = -zero_turn_omega;
        v =0;
        error =0;
    elseif (state==7)
        error = pose(2)-1;
        v=speed_straight;
        omega = 0;
        last_errors(1) = 0;
        last_errors(2) = 0;
        last_errors(3) = 0;
    elseif (state == 7.5)
        omega = -zero_turn_omega;
        v =0;
        error =0;
        pose(3)
    elseif (state == 8)
        error = pose(1)-0;
        v=speed_straight;
        omega = 0;
        last_errors(1) = 0;
        last_errors(2) = 0;
        last_errors(3) = 0;
    elseif (state == 9)
         robot.setvel(0,0);
        ellipseTracking(robot);
   end
    
    %%%%PID Controller%%%%%%%
    if last_error == 0
        change_in_error = 0;
    else
change_in_error = error-last_error;
    end
    last_errors(3) = last_errors(2);
    last_errors(2) = last_errors(1);
    last_errors(1) = change_in_error;
    last_error = error;
    accumulated_error = accumulated_error+error;
   omega_correction = error*abs(error*error)*kp + (accumulated_error*ki)+(kd*change_in_error*dt); %%TAMARA CHANGED TO MULT DT
  %  omega_correction = error*kp + (accumulated_error*ki)+(change_in_error*kd*dt) %%TAMARA CHANGED TO MULT DT
    if omega_correction > .1
        omega_correction =.1;
    end

    
    %%%%EKF%%%%%%
    % Time Update
    error
    v_corrupted = v - v_correction;
    omega_corrupted = omega - omega_correction;
   
    
    wheel = int16([0.5, 0.5; 1/.3, -1/.3] \ [v_corrupted; omega_corrupted] .* 1000);
            if -500 > wheel(1) || wheel(1) > 500 ||  -500 > wheel(2) || wheel(2) > 500
                %correction too big
                ahhhh = 'AHHHHH'
                omega_corrupted = last_omega_corrupted;
            else
               last_omega_corrupted = omega_corrupted; 
            end
             robot.setvel(v_corrupted,omega_corrupted);
    %Calculate A and W
    theta = x(3);
    A = [1,0,-v_corrupted*sin(theta)*dt;0,1,v_corrupted*cos(theta)*dt;0,0,1]; %In the future change these to v+correction
    W = [cos(theta)*dt,0;sin(theta)*dt,0;0,dt];
    
    %Update our state estimate
   
    x = x + [v_corrupted*cos(theta)*dt;v_corrupted*sin(theta)*dt;omega_corrupted*dt];
    %Update the covariance estimate P
    P = A*P*A' + W*Q*W';
    
    % Measurement Update

   [rho, alpha] = scan_real_lidar(invMat,dispx,lidar, lc, agg);
    if (isnan(rho) || isnan(alpha))
        alarm = 'No LIDAR'
       continue;
    end
 if (v == 0)
     continue;
 end
    
    %Calculate H
    H = [(x(1)-lidar(1))/sqrt((x(1)-lidar(1))^2+(x(2)-lidar(2))^2), (x(2)-lidar(2))/sqrt((lidar(1)-x(1))^2+(lidar(2)-x(2))^2), 0;
        -x(2)/(x(1)^2+x(2)^2), x(1)/(x(1)^2+x(2)^2), 0];
    
    %Calculate V
    V = [1,0;0,1];
    %Compute Kalman Gain
    K = P*H'*((H*P*H'+V*R*V')^-1);
    %State Update
    odom_angle=atan2(x(2),x(1));
    angle_difference = atan2(sin(alpha-odom_angle), cos(alpha-odom_angle));
   
    x = x + K*([rho;angle_difference] - [sqrt((x(1)-lidar(1))^2+(x(2)-lidar(2))^2);0]);
   %Covariance Update
    P = (eye(3)-K*H)*P;
    
    % We need this to flush the I/O so it plots nicely
    pause(0.01);
end
    if state == 8
        ellipseTracking(robot);
    end

disp('Simulation finished.  Save your figures if necessary and hit any key to continue...')
pause();
end



% Implement this function IAW B.2.c of the assignment
function [rho, alpha] = scan_lidar( lidar, robot, R )
% *** ADD NECESSARY CODE HERE ***
%This function simulates a lidar measurement returning a range and bearing
%in the LIDAR frame. These measurements are courupted with zero-mean
%Gaussian noise using the covariance matrix provided(R)
%
%lidar is a 3x1 array with the position and orientation of the lidar in the
%global fram
%robot is the irobot create object
%R is the measurement covariance matrix which we will use to courupt the
%values

%Get the x,y,theta of the robot in the global frame
position = robot.getpose();
%Throw away the theta measurement from getpose() as the lidar cannot
%determine the theta of the robot
position_Global_Frame = [position(1); position(2)];
%Use a 2d rotation matrix to get the values in the Lidar coordinate frame
%and translate them to get them in the right position relative to the LIDAR
position_Lidar_Frame = rot2d(lidar(3))'*position_Global_Frame;         %%TODO: Make sure that this counterclockwise rot makes sense
position_Lidar_Frame = position_Lidar_Frame-[lidar(1); lidar(2)];      %%TODO: Make sure that this sumbtraction makes sense
%Convert the values from Cartesian to Polar Coordinates
position_Polar = [sqrt((position_Lidar_Frame(1)^2)+(position_Lidar_Frame(2)^2)); atan2(position_Lidar_Frame(2),position_Lidar_Frame(1))];
%Finally courrupt with zero mean Gaussian noise IAW the R matrix provided
rho = sqrt((position(1)-lidar(1))^2+(position(2)-lidar(2))^2)+ (sqrt(R(1,1)) * randn(1,1));
%rho = position_Polar(1)+normrnd(0,sqrt(R(1,1)));
alpha = position_Polar(2)+(sqrt(R(2,2)) * randn(1,1));
end

% function R = rot2d(theta)
% - Generate a 2D rotation matrix for a given angle theta
function R = rot2d(theta)
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
end
