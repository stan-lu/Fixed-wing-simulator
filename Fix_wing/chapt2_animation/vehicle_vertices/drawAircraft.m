
function drawAircraft(uu,V,F,patchcolors)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);           
    u        = uu(4);       
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);       % roll rate
    q        = uu(11);       % pitch rate     
    r        = uu(12);       % yaw rate    
    t        = uu(13);       % time

    % define persistent variables 
    persistent vehicle_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1), clf
        [Vertices,Faces,facecolors] = defineAircraftBody;
        vehicle_handle = drawVehicleBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Vehicle')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis([-100,100,-100,100,-100,100]);
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawVehicleBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           vehicle_handle);
    end
end

  
%=======================================================================
% drawVehicle
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawVehicleBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  V = rotate(V, phi, theta, psi);  % rotate vehicle
%   V = translate(V, pn, pe, pd);  % translate vehicle
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = R*V;
  
  if isempty(handle)
  handle = patch('Vertices', V', 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V','Faces',F);
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,phi,theta,psi)

  % define rotation matrix (right handed)
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;  
    % note that R above either leaves the vector alone or rotates
    % a vector in a left handed rotation.  We want to rotate all
    % points in a right handed rotation, so we must transpose
  R = R';

  % rotate vertices
  pts = R*pts;
  
end
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

  pts = pts + repmat([pn;pe;pd],1,size(pts,2));
  
end

% end translate


%=======================================================================
% defineVehicleBody
%=======================================================================
function [V,F,facecolors] = defineVehicleBody

% Define the vertices (physical location of vertices
V = 10*[...
    1, 0, 0;...   % pt 1
    -1, -2, 0;... % pt 2
    0, 0, 0;...   % pt 3
    -1, 2, 0;...  % pt 4
    0, 0, -1;...  % pt 5
    ]';

% define faces as a list of vertices numbered above
  F = [...
        1, 2, 3;...  % left wing
        1, 3, 4;...  % right wing
        1, 3, 5;...  % tail 
        ];

% define colors for each face    
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mycyan = [0, 1, 1];

  facecolors = [...
    mygreen;...    % left wing
    mygreen;...    % right wing
    myblue;...     % tail
    ];
end
% define aircraft vertices and faces
function [V,F,colors] = defineAircraftBody

% parameters for drawing aircraft
  % scale size
  size = 2;
  fuse_l1    = 7;
  fuse_l2    = 4;
  fuse_l3    = 15;
  fuse_w     = 2;
  wing_l     = 6;
  wing_w     = 20;
  tail_l     = 3;
  tail_h     = 3;
  tailwing_w = 10;
  tailwing_l = 3;
  % colors
  red     = [1, 0, 0];
  green   = [0, 1, 0];
  blue    = [0, 0, 1];
  yellow  = [1,1,0];
  magenta = [0, 1, 1];
  

% define vertices and faces for aircraft
  V = [...
    fuse_l1,             0,             0;...        % point 1
    fuse_l2,            -fuse_w/2,     -fuse_w/2;... % point 2     
    fuse_l2,             fuse_w/2,     -fuse_w/2;... % point 3     
    fuse_l2,             fuse_w/2,      fuse_w/2;... % point 4
    fuse_l2,            -fuse_w/2,      fuse_w/2;... % point 5
   -fuse_l3,             0,             0;...        % point 6
    0,                   wing_w/2,      0;...        % point 7
   -wing_l,              wing_w/2,      0;...        % point 8
   -wing_l,             -wing_w/2,      0;...        % point 9
    0,                  -wing_w/2,      0;...        % point 10
   -fuse_l3+tailwing_l,  tailwing_w/2,  0;...        % point 11
   -fuse_l3,             tailwing_w/2,  0;...        % point 12
   -fuse_l3,            -tailwing_w/2,  0;...        % point 13
   -fuse_l3+tailwing_l, -tailwing_w/2,  0;...        % point 14
   -fuse_l3+tailwing_l,  0,             0;...        % point 15
   -fuse_l3+tailwing_l,  0,             -tail_h;...  % point 16
   -fuse_l3,             0,             -tail_h;...  % point 17
  ]';
  
  F = [...
        1,  2,  3,  1;... % nose-top
        1,  3,  4,  1;... % nose-left
        1,  4,  5,  1;... % nose-bottom
        1,  5,  2,  1;... % nose-right
        2,  3,  6,  2;... % fuselage-top
        3,  6,  4,  3;... % fuselage-left
        4,  6,  5,  4;... % fuselage-bottom
        2,  5,  6,  2;... % fuselage-right
        7,  8,  9, 10;... % wing
       11, 12, 13, 14;... % tailwing
        6, 15, 17, 17;... % tail
        
  ];  
  
  colors = [...
        yellow;... % nose-top
        yellow;... % nose-left
        yellow;... % nose-bottom
        yellow;... % nose-right
        blue;... % fuselage-top
        blue;... % fuselage-left
        red;... % fuselage-bottom
        blue;... % fuselage-right
        green;... % wing
        green;... % tailwing
        blue;... % tail
    ];

  V = size*V;   % rescale vertices

end
  
  