 
function rot = Euler2R(ang)
%	Converts an euler angles vector [phi,theta,psi] into a rotation matrix
%	of the Z-Y-X representation: R = rot_z(psi)*rot_y(theta)*rot_x(phi).

rotx = [	1	,	0			,	 0
			0	,	cos(ang(1))	,	-sin(ang(1))
			0	,	sin(ang(1))	,	 cos(ang(1))	];
roty = [	 cos(ang(2))	,	0	,	sin(ang(2))
			0				,	1	,	0
			-sin(ang(2))	,	0	,	cos(ang(2))	];
rotz = [	cos(ang(3))	,	-sin(ang(3))	,	0
			sin(ang(3))	,	 cos(ang(3))	,	0
			0			,	 0				,	1	];
        
%rotation matrix
rot = rotz*roty*rotx;

% orientation of frame B in frame W
W_Rb = [cos(ang(3))*cos(ang(2))-sin(ang(1))*sin(ang(2))*sin(ang(3)) -cos(ang(1))*sin(ang(3)) cos(ang(3))*sin(ang(2))+cos(ang(2))*sin(ang(1))*sin(ang(3))
        cos(ang(2))*sin(ang(3))+cos(ang(3))*sin(ang(1))*sin(ang(2))  cos(ang(1))*cos(ang(3)) sin(ang(3))*sin(ang(2))-cos(ang(3))*cos(ang(2))*sin(ang(1))
       -cos(ang(1))*sin(ang(2))                                          sin(ang(1))         cos(ang(1))*cos(ang(2))                      ];
   
   
% angular velocity
Wb  = [cos(ang(2)) 0 -cos(ang(2)); 0 1 sin(ang(1)); sin(ang(2)), 0, cos(ang(1))*cos(ang(2)) ] * [ang(1);ang(2);ang(3)];




