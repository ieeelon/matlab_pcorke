%-----------------------------------------------------------------------
% slCharacterEncoding('ISO-8859-1'); % If you use Matlab R2014a uncomment
% this line
%-----------------------------------------------------------------------

%MDL_PUMA560 Create model of Puma 560 manipulator
%
%      mdl_puma560
%
% Script creates the workspace variable p560 which describes the 
% kinematic and dynamic characteristics of a Unimation Puma 560 manipulator
% using standard DH conventions.
% The model includes armature inertia and gear ratios.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         vertical 'READY' configuration
%   qstretch   arm is stretched out in the X direction
%   qn         arm is at a nominal non-singular configuration
%
% Reference::
% - "A search for consensus among model parameters reported for the PUMA 560 robot",
%   P. Corke and B. Armstrong-Helouvry, 
%   Proc. IEEE Int. Conf. Robotics and Automation, (San Diego), 
%   pp. 1608-1613, May 1994.
%
% See also SerialRevolute, mdl_puma560akb, mdl_stanford, mdl_twolink.

%
% Notes:
%    - the value of m1 is given as 0 here.  Armstrong found no value for it
% and it does not appear in the equation for tau1 after the substituion
% is made to inertia about link frame rather than COG frame.
% updated:
% 2/8/95  changed D3 to 150.05mm which is closer to data from Lee, AKB86 and Tarn
%  fixed errors in COG for links 2 and 3
% 29/1/91 to agree with data from Armstrong etal.  Due to their use
%  of modified D&H params, some of the offsets Ai, Di are
%  offset, and for links 3-5 swap Y and Z axes.
% 14/2/91 to use Paul's value of link twist (alpha) to be consistant
%  with ARCL.  This is the -ve of Lee's values, which means the
%  zero angle position is a righty for Paul, and lefty for Lee.
%  Note that gravity load torque is the motor torque necessary
%  to keep the joint static, and is thus -ve of the gravity
%  caused torque.
%
% 8/95 fix bugs in COG data for Puma 560. This led to signficant errors in
%  inertia of joint 1. 
% $Log: not supported by cvs2svn $
% Revision 1.4  2008/04/27 11:36:54  cor134
% Add nominal (non singular) pose qn



% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com
%
%
%
%modified for the lab purposes by Madi Nurmanov https://github.com/ieeelon/matlab_pcorke

clear L
deg = pi/180;


%Intertia matrix calculation
m1 = 0; m2 = 10; m3 = 10;
m = [m1, m2, m3];
I = 10^(-4);
l = 1;
R = 0.05;

i = 1;
cI1 = [0.5*m(i)*R^2, ... 
        ((1/12)*m(i)*l^2 + 0.25*m(i)*R^2), ...
        ((1/12)*m(i)*l^2 + 0.25*m(i)*R^2)];
i = 2;
cI2 = [0.5*m(i)*R^2, ... 
        ((1/12)*m(i)*l^2 + 0.25*m(i)*R^2), ...
        ((1/12)*m(i)*l^2 + 0.25*m(i)*R^2)];
i = 3;
cI3 = [0.5*m(i)*R^2, ... 
        ((1/12)*m(i)*l^2 + 0.25*m(i)*R^2), ...
        ((1/12)*m(i)*l^2 + 0.25*m(i)*R^2)];
    
Bfr = 10*10^(-4);
% joint angle limits from 
% A combined optimization method for solving the inverse kinematics problem...
% Wang & Chen
% IEEE Trans. RA 7(4) 1991 pp 489-
L(1) = Revolute('modified', 'd', 0, 'a', 0, 'alpha', 0, ...
    'I', cI1, ...
    'r', [0, 0, 0], ...
    'm', m1, ...
    'Jm', 10^(-4), ...
    'G', 500, ...
    'B', Bfr, ...
    'Tc', [0.395 -0.435], ...
    'qlim', [-180 180]*deg );


L(2) = Revolute('modified', 'd', 0, 'a', 0, 'alpha', -pi/2, ...
    'I', cI1, ...
    'r', [0, 0, 0], ...
    'm', m1, ...
    'Jm', 10^(-4), ...
    'G', 500, ...
    'B', Bfr, ...
    'Tc', [0.395 -0.435], ...
    'qlim', [-180 180]*deg );

L(3) = Revolute('modified', 'd', 0, 'a', 1, 'alpha', 0, ...
    'I', cI2, ...
    'r', [-0.3638, 0.006, 0.2275], ...
    'm', m2, ...
    'Jm', 10^(-4), ...
    'G', 500, ...
    'B', Bfr, ...
    'Tc', [0.126 -0.071], ...
    'qlim', [-90 90]*deg );

L(4) = Revolute('modified', 'd', 0, 'a', 1, 'alpha', 0,  ...
    'm', 1);

L1(1) = Revolute('d', 1, 'a', 0, 'alpha', -pi/2, ...
    'I', cI1, ...
    'r', [0, 0, 0], ...
    'm', m1, ...
    'Jm', 10^(-4), ...
    'G', 500, ...
    'B', Bfr, ...
    'Tc', [0.395 -0.435], ...
    'qlim', [-180 180]*deg );

L1(2) = Revolute('d', 0, 'a', 1, 'alpha', 0, ...
    'I', cI2, ...
    'r', [-0.3638, 0.006, 0.2275], ...
    'm', m2, ...
    'Jm', 10^(-4), ...
    'G', 500, ...
    'B', Bfr, ...
    'Tc', [0.126 -0.071], ...
    'qlim', [-90 90]*deg );

L1(3) = Revolute('d', 0, 'a', 1, 'alpha', 0,  ...
    'I', cI3, ...
    'r', [-0.0203, -0.0141, 0.070], ...
    'm', m3, ...
    'Jm', 10^(-4), ...
    'G', 500, ...
    'B', Bfr, ...
    'Tc', [0.132, -0.105], ...
    'qlim', [-90 90]*deg );

L1(4) = Revolute('d', 0, 'a', 0, 'alpha', 0,  ...
    'm', 1);

p560beta = SerialLink(L, 'name', 'Puma 560beta', ...
    'manufacturer', 'Unimation', 'ikine', 'puma', 'comment', 'viscous friction; params of 8/95');

p560alpha = SerialLink(L1, 'name', 'Puma 560alpha', ...
    'manufacturer', 'Unimation', 'ikine', 'puma', 'comment', 'viscous friction; params of 8/95');
%
% some useful poses
%
qz = [0 0 0]; % zero angles, L shaped pose
qr = [0 -pi/2 0]; % ready pose, arm up
qs = [0 0 -pi/2];
qn=[0 pi/4 -pi/2];

homeb = [0 0 pi/2 0];
holdb = [0 0 0 0];

homea = [0 0 pi/2 0];
holda = [0 0 0 0];

% p560beta.teach()
% p560alpha.teach()
% 
% p560beta.plot(homeb)
% p560beta.plot(holdb)
% p560alpha.plot(homea)
% p560alpha.plot(holda)
% 
% p560beta.gravload(homeb)
% p560beta.gravload(holdb)
% p560alpha.gravload(homea)
% p560alpha.gravload(holda)
% 
% p560beta.jacob0(homeb)
% p560beta.jacob0(holdb)
% p560alpha.jacob0(homea)
% p560alpha.jacob0(holda)

clear L
