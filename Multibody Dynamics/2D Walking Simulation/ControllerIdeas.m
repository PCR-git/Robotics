% Various Controller Ideas for the Reduced-DOF Snake Robot

%% Motor Torques

% Needed motor torques, given desired joint torques
% Controller 1, Link Angle Average
% [TorqueM1, TorqueM2] = MotorTorque(Uvec,MechData);

% % Controller 2, Simple PD
% TorqueM1 = Amp*(P*((A*sin(w*t+offset-0*d))+phinought-(phi4-phi1))+D*w*(A*cos(w*t+offset-0*d)-(phi4p-phi1p)));
% TorqueM2 = Amp*(P*((A*sin(w*t+offset-1*d))+phinought-(phi8-phi5))+D*w*(A*cos(w*t+offset-1*d)-(phi8p-phi5p)));

% % Simple P
% TorqueM1 = Amp*(P*((A*sin(w*t+offset-0*d))+phinought-(phi4-phi1)));
% TorqueM2 = Amp*(P*((A*sin(w*t+offset-1*d))+phinought-(phi8-phi5)));

% Controller 3, weird1
% TorqueM1 = Amp*(P*(sin(A*(cos(w*t)+offset-0*d))+phinought-(phi4-phi1))-D*(A*w*sin(w*t)*cos(A*(cos(w*t)+offset-0*d))-(phi4p-phi1p)));
% TorqueM2 = Amp*(P*(sin(A*(cos(w*t)+offset-1*d))+phinought-(phi8-phi5))-D*(A*w*sin(w*t)*cos(A*(cos(w*t)+offset-1*d))-(phi8p-phi5p)));

% TorqueM1 = Amp*(P*((1/sin(1/2))*sin(A*(0.5*cos(w*t-0*d)-0*d)+phinought)-(phi4-phi1)));
% TorqueM2 = Amp*(P*((1/sin(1/2))*sin(A*(0.5*cos(w*t-1*d)-0*d)+phinought)-(phi8-phi5)));

% % Controller 4, weird2
% TorqueM1 = Amp*(P*((1/sin(1/2))*sin(A*(0.5*sin(w*t-0*d)-0*d)+phinought)-(phi4-phi1)));
% TorqueM2 = Amp*(P*((1/sin(1/2))*sin(A*(0.5*sin(w*t-1*d)-0*d)+phinought)-(phi8-phi5)));

% % Controller 5, WTF is this anyway?
% Fact1 = -u1/2;
% Fact4 = u3/2-u4/2;
% Fact5 = u4/2-u5/2;
% Fact8 = u7/2;
% TorqueM1 = (Fact4-Fact1);
% TorqueM2 = (Fact8-Fact5);

% % Controller 6, FAILED
% TorqueM1 = 2*(Fact4-Fact1)/(LinkRadius/Pradius3-LinkRadius/Pradius1);
% TorqueM2 = 2*(Fact8-Fact5)/(LinkRadius/Pradius3-LinkRadius/Pradius1);

% Controller 7, Amplitude Controller
theta1 = phi2-phi1;
theta2 = phi3-phi2;
theta5 = phi6-phi5;
theta6 = phi7-phi6;
amp1 = cos((pi-theta2)/2)+cos((pi-theta2)/2-theta1);
amp2 = cos((pi-theta6)/2)+cos((pi-theta6)/2-theta5);

theta1p = phi2p-phi1p;
theta2p = phi3p-phi2p;
theta5p = phi6p-phi5p;
theta6p = phi7p-phi6p;

amp1p = (theta2p/2)*sin((pi-theta2)/2)+(theta2p/2+theta1p)*sin((pi-theta2)/2-theta1);
amp2p = (theta6p/2)*sin((pi-theta6)/2)+(theta6p/2+theta5p)*sin((pi-theta6)/2-theta5);

% P Amplitude Controller
TorqueM1 = Amp*(P*((1/sin(1/2))*sin(A*(0.5*sin(w*t-0*d))+phinought)-amp1));
TorqueM2 = Amp*(P*((1/sin(1/2))*sin(A*(0.5*sin(w*t-1*d))+phinought)-amp2));

% % PD Amplitude Controller
% TorqueM1 = Amp*(P*((1/sin(1/2))*sin(A*(0.5*sin(w*t-0*d))+phinought)-amp1)+D*((1/sin(1/2))*0.5*A*w*cos(0*d-w*t)*cos(0.5*A*sin(0*d-w*t))-amp1p));
% TorqueM2 = Amp*(P*((1/sin(1/2))*sin(A*(0.5*sin(w*t-1*d))+phinought)-amp2)+D*((1/sin(1/2))*0.5*A*w*cos(1*d-w*t)*cos(0.5*A*sin(1*d-w*t))-amp2p));
