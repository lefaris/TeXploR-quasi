% Lauren Ervin
% Agile Robotics Laboratory at UA
% TExploR Project
% Date: 02/26/24
% 
% This class takes in theta1, theta2 values (one for each arc).  It calls
% the TExploRDerivationsFunc class to derive phi_i, zB, and Fri values.  
% The resulting 4 arc combinations are then plotted for visualization.
%
% This is achieved through the geometric relationship between the
% two arcs, the holonomic constraints that stick the  points of 
% contact to the ground plane at all times, and the statics.
%
% Usage:
% Create an instance of the TExploRStaticsDerivations class with the
% desired theta1 and theta2 values to generate simulated phi_i values:
% TExploRStaticsDerivations(0, 90);

classdef TExploRStaticsDerivationsQuasi < handle

    properties
        T12
        R12
        gc
        m
        M
        rB
        zs
        theta1
        theta2
        endpoint1
        endpoint2
        staticPoses
        r
        tt
        increment
        iter
        force
        state
    end
    methods(Static)
        function ang = wrapTozero2Pi(x) % Constrain between 0 and 2pi
            ang = (1 - sign(x)) * pi / 2 + x;
        end
%         Point on the arc in the arc C.S. given the radius r, angle alpha
        function p = ptonlink3(r, alpha_rad) % Calculate point on arc
            p = r * [cos(alpha_rad), sin(alpha_rad), 0]';
        end        
%         Tangent of the point on the arc in arc C.S.
        function t = ptonlinkTangent3(r, alpha_rad) % Calculate point on arc tangent
            t = r * [-sin(alpha_rad), cos(alpha_rad), 0]';
        end
%         The rotation matrix given two free vectors zB and zs
        function R = getRMatnumeric(zB, zs) % Calculate rotation matrix from zB to zs
%             zB = zB/norm(zB);
            omega = LieGroup.vec2so3(zB)*zs; % Axis of rotation
            % theta12 = atan2(omegaNorm,v1'*v2);
            ctheta = zB'*zs/(norm(zB)*norm(zs));
            stheta = norm(omega);
            theta = atan2(stheta,ctheta);
            omega = omega/norm(omega);
            R = LieGroup.MatExponential3(omega,theta);
        end
    end
    methods
        function obj = TExploRStaticsDerivationsQuasi(theta1_deg, theta2_deg) % Only need theta1, theta2 inputs
%             1 - Relationship between the two arc
            obj.T12 = [0,0,1,0; 0,-1,0,0; 1, 0, 0, 0; 0, 0, 0, 1]; % Transformation matrix between the two arcs
            %obj.T12 = [0,1,0,0; 0,0,-1,0.06; -1, 0, 0, 0; 0, 0, 0, 1]; % Transformation matrix between the two arcs
            obj.R12 = obj.T12(1:3,1:3); % Rotation matrix between the two arcs
%             2 - Other robot parameters
            obj.gc = 9.81; % Gravity constant            
            obj.m = 0.5; % 200g shifting mass weight - may need to change (FIXME)  FLIPPED THESE FOR TESTING
            obj.M = 0.4; % 300g arc weight - may need to change (FIXME)
            obj.rB = 0.2; % 0.15m distance from center - may need to change (FIXME)
            obj.zs = [0;0;1]; % zs hat
            
            
            
            %%%%%%%%%%%%%%QUASISTATIC CHANGES%%%%%%%%%%%%%%%%%%%%%%
            obj.endpoint1 = deg2rad(theta1_deg); % Convert theta1 to radians
            obj.endpoint2 = deg2rad(theta2_deg); % Convert theta2 to radians
            obj.theta1 = deg2rad(theta1_deg); % Convert theta1 to radians
            obj.theta2 = deg2rad(theta2_deg); % Convert theta2 to radians



            obj.r = 1; % Radius of arc from center
            obj.staticPoses = struct('Case', []); % Create two cases of only t1b or t2b existing
            obj.increment = 1;
            obj.iter = 1; %Iterate for valid forces check
            obj.computeStaticPoses();
        end

        function staticPoses = computeStaticPoses(obj)
            %close   
            figure(1)
%            f = figure(2)
%            f.Position = [100 100 1000 700];
%             for j = 0:10:180
%                 obj.theta1 = deg2rad(j);
                for ii = 1:1:4    
                    % Case 2 (only t1b exists)
                    obj.increment = ii;
%                     if ii == 1
%                         ii
%                          for j = 1:10:171
% %                        for j = 171:-10:1
%                             %%%%%%%%%phi2 = 0, theta 2 = 0, theta1 = 0:180
%                             obj.theta2 = deg2rad(0);
%                             obj.theta1 = deg2rad(j);
%                             % Case 2a phi2 = 0
%                             [obj.staticPoses.Case(ii).phi1, obj.staticPoses.Case(ii).zB, obj.staticPoses.Case(ii).Fr1, obj.staticPoses.Case(ii).Fr2] = TExploRDerivationsFunc.c2a(obj.m,obj.M,obj.gc,obj.theta1,obj.theta2);
%                             obj.staticPoses.Case(ii).phi2 = 0;
% 
% 
%                             %%%%%%%%%%%%%ADDING
%                             % Calculate qb1, qb2, and Tsb for two sets of phi1 and phi2 values
%                             obj.staticPoses.Case(ii).qB1 = obj.ptonlink3(obj.r, obj.staticPoses.Case(ii).phi1); %Changed obj.r to obj.rB
%                             obj.staticPoses.Case(ii).qB2 = obj.R12 * obj.ptonlink3(obj.r, obj.staticPoses.Case(ii).phi2); %Changed obj.r to obj.rB
%                             obj.staticPoses.Case(ii).Rsb = obj.getRMatnumeric(obj.staticPoses.Case(ii).zB, obj.zs);
%                             obj.staticPoses.Case(ii).psb = [0; 0; obj.r/sqrt(2)];
%                             obj.staticPoses.Case(ii).Tsb = [obj.staticPoses.Case(ii).Rsb, obj.staticPoses.Case(ii).psb; 0, 0, 0, 1];
%                             obj.quasiStatic(obj.staticPoses.Case(ii).Tsb,obj.theta1,obj.theta2,obj.staticPoses.Case(ii).phi1,obj.staticPoses.Case(ii).phi2, obj.increment);
%                             obj.iter = obj.iter + 1;
% %                            staticPoses = obj.staticPoses;
%                         end
                    %end
                        
%                     elseif ii == 2  
                     if ii == 1
%                         for u = 1:10:171
%                             %%%%%%phi1 = 0, theta1 = 0, theta2 = 180:0
%                             obj.theta1 = deg2rad(0);
%                             obj.theta2 = deg2rad(u);
                        for h = 1:10:171
                            %%%%%%%%%phi1 = 180, theta1 = 180, theta2 = 0:180
                            obj.theta1 = deg2rad(0);
                            obj.theta2 = deg2rad(h);
%                         % Case 2b phi2 = pi
%                         [obj.staticPoses.Case(ii).phi1, obj.staticPoses.Case(ii).zB, obj.staticPoses.Case(ii).Fr1, obj.staticPoses.Case(ii).Fr2] = TExploRDerivationsFunc.c2b(obj.m,obj.M,obj.gc,obj.theta1,obj.theta2);
%                         obj.staticPoses.Case(ii).phi2 = pi;
                        
                        
                            %%%%%%%%%%%%ADDING
                            % Case 3b phi1 = pi
%                             [obj.staticPoses.Case(ii).phi2, obj.staticPoses.Case(ii).zB, obj.staticPoses.Case(ii).Fr1, obj.staticPoses.Case(ii).Fr2] = TExploRDerivationsFunc.c3b(obj.m,obj.M,obj.gc,obj.theta1,obj.theta2);
%                             obj.staticPoses.Case(ii).phi1 = pi;
                            [obj.staticPoses.Case(ii).phi2, obj.staticPoses.Case(ii).zB, obj.staticPoses.Case(ii).Fr1, obj.staticPoses.Case(ii).Fr2] = TExploRDerivationsFunc.c3a(obj.m,obj.M,obj.gc,obj.theta1,obj.theta2);
                            obj.staticPoses.Case(ii).phi1 = 0;

                                % Calculate qb1, qb2, and Tsb for two sets of phi1 and phi2 values
                            obj.staticPoses.Case(ii).qB1 = obj.ptonlink3(obj.r, obj.staticPoses.Case(ii).phi1); %Changed obj.r to obj.rB
                            obj.staticPoses.Case(ii).qB2 = obj.R12 * obj.ptonlink3(obj.r, obj.staticPoses.Case(ii).phi2); %Changed obj.r to obj.rB
                            obj.staticPoses.Case(ii).Rsb = obj.getRMatnumeric(obj.staticPoses.Case(ii).zB, obj.zs);
                            obj.staticPoses.Case(ii).psb = [0; 0; obj.r/sqrt(2)];
                            %obj.staticPoses.Case(ii).psb = [1; 1; obj.r/sqrt(2)];
                            obj.staticPoses.Case(ii).Tsb = [obj.staticPoses.Case(ii).Rsb, obj.staticPoses.Case(ii).psb; 0, 0, 0, 1];
                            obj.quasiStatic(obj.staticPoses.Case(ii).Tsb,obj.theta1,obj.theta2,obj.staticPoses.Case(ii).phi1,obj.staticPoses.Case(ii).phi2, obj.increment);
                            obj.iter = obj.iter + 1;
%                            staticPoses = obj.staticPoses;
                        end
                        
                    %end

                    % Case 3 (only t2b exists)
%                     elseif ii == 3
                    elseif ii == 2
                        for l = 1:10:171
                            %%%%%%phi2 = 180, theta2 = 180, theta1 = 180:0
                            obj.theta2 = deg2rad(180);
                            obj.theta1 = deg2rad(l);
%                         % Case 3a phi1 = 0
%                         [obj.staticPoses.Case(ii).phi2, obj.staticPoses.Case(ii).zB, obj.staticPoses.Case(ii).Fr1, obj.staticPoses.Case(ii).Fr2] = TExploRDerivationsFunc.c3a(obj.m,obj.M,obj.gc,obj.theta1,obj.theta2);
%                         obj.staticPoses.Case(ii).phi1 = 0;
                        
                        
                            %%%%%%%%%%%%ADDING
                            % Case 2b phi2 = pi
                            [obj.staticPoses.Case(ii).phi1, obj.staticPoses.Case(ii).zB, obj.staticPoses.Case(ii).Fr1, obj.staticPoses.Case(ii).Fr2] = TExploRDerivationsFunc.c2b(obj.m,obj.M,obj.gc,obj.theta1,obj.theta2);
                            obj.staticPoses.Case(ii).phi2 = pi;

                            % Calculate qb1, qb2, and Tsb for two sets of phi1 and phi2 values
                            obj.staticPoses.Case(ii).qB1 = obj.ptonlink3(obj.r, obj.staticPoses.Case(ii).phi1); %Changed obj.r to obj.rB
                            obj.staticPoses.Case(ii).qB2 = obj.R12 * obj.ptonlink3(obj.r, obj.staticPoses.Case(ii).phi2); %Changed obj.r to obj.rB
                            obj.staticPoses.Case(ii).Rsb = obj.getRMatnumeric(obj.staticPoses.Case(ii).zB, obj.zs);
                            obj.staticPoses.Case(ii).psb = [0; 0; obj.r/sqrt(2)];
                            %obj.staticPoses.Case(ii).psb = [1; 1; obj.r/sqrt(2)];
                            obj.staticPoses.Case(ii).Tsb = [obj.staticPoses.Case(ii).Rsb, obj.staticPoses.Case(ii).psb; 0, 0, 0, 1];
                            obj.quasiStatic(obj.staticPoses.Case(ii).Tsb,obj.theta1,obj.theta2,obj.staticPoses.Case(ii).phi1,obj.staticPoses.Case(ii).phi2, obj.increment);
                            obj.iter = obj.iter + 1;
%                            staticPoses = obj.staticPoses;
                        end
                    %end
                        
%                     elseif ii == 4
                    elseif ii == 3
                        for h = 171:-10:1
                            %%%%%%%%%phi1 = 180, theta1 = 180, theta2 = 0:180
                            obj.theta1 = deg2rad(180);
                            obj.theta2 = deg2rad(h);
%                         for u = 180:30:0
%                             %%%%%%phi1 = 0, theta1 = 0, theta2 = 180:0
%                             obj.theta1 = deg2rad(0);
%                             obj.theta2 = deg2rad(u);
%                         % Case 3b phi1 = pi
%                         [obj.staticPoses.Case(ii).phi2, obj.staticPoses.Case(ii).zB, obj.staticPoses.Case(ii).Fr1, obj.staticPoses.Case(ii).Fr2] = TExploRDerivationsFunc.c3b(obj.m,obj.M,obj.gc,obj.theta1,obj.theta2);
%                         obj.staticPoses.Case(ii).phi1 = pi;

                          %%%%%%%%%%%%%%%%ADDING
                          % Case 3a phi1 = 0
%                             [obj.staticPoses.Case(ii).phi2, obj.staticPoses.Case(ii).zB, obj.staticPoses.Case(ii).Fr1, obj.staticPoses.Case(ii).Fr2] = TExploRDerivationsFunc.c3a(obj.m,obj.M,obj.gc,obj.theta1,obj.theta2);
%                             obj.staticPoses.Case(ii).phi1 = 0;
                            [obj.staticPoses.Case(ii).phi2, obj.staticPoses.Case(ii).zB, obj.staticPoses.Case(ii).Fr1, obj.staticPoses.Case(ii).Fr2] = TExploRDerivationsFunc.c3b(obj.m,obj.M,obj.gc,obj.theta1,obj.theta2);
                             obj.staticPoses.Case(ii).phi1 = pi;

                            % Calculate qb1, qb2, and Tsb for two sets of phi1 and phi2 values
                            obj.staticPoses.Case(ii).qB1 = obj.ptonlink3(obj.r, obj.staticPoses.Case(ii).phi1); %Changed obj.r to obj.rB
                            obj.staticPoses.Case(ii).qB2 = obj.R12 * obj.ptonlink3(obj.r, obj.staticPoses.Case(ii).phi2); %Changed obj.r to obj.rB
                            obj.staticPoses.Case(ii).Rsb = obj.getRMatnumeric(obj.staticPoses.Case(ii).zB, obj.zs);
                            obj.staticPoses.Case(ii).psb = [0; 0; obj.r/sqrt(2)];
                            %obj.staticPoses.Case(ii).psb = [1; 1; obj.r/sqrt(2)];
                            obj.staticPoses.Case(ii).Tsb = [obj.staticPoses.Case(ii).Rsb, obj.staticPoses.Case(ii).psb; 0, 0, 0, 1];
                            obj.quasiStatic(obj.staticPoses.Case(ii).Tsb,obj.theta1,obj.theta2,obj.staticPoses.Case(ii).phi1,obj.staticPoses.Case(ii).phi2, obj.increment);
                            obj.iter = obj.iter + 1;
%                            staticPoses = obj.staticPoses;
                        end
                    
                        
                     elseif ii == 4
                         for j = 1:10:171
%                        for j = 171:-10:1
                            %%%%%%%%%phi2 = 0, theta 2 = 0, theta1 = 0:180
                            obj.theta2 = deg2rad(180);
                            obj.theta1 = deg2rad(j);
                            % Case 2a phi2 = 0
                            [obj.staticPoses.Case(ii).phi1, obj.staticPoses.Case(ii).zB, obj.staticPoses.Case(ii).Fr1, obj.staticPoses.Case(ii).Fr2] = TExploRDerivationsFunc.c2b(obj.m,obj.M,obj.gc,obj.theta1,obj.theta2);
                            obj.staticPoses.Case(ii).phi2 = pi;


                            %%%%%%%%%%%%%ADDING
                            % Calculate qb1, qb2, and Tsb for two sets of phi1 and phi2 values
                            obj.staticPoses.Case(ii).qB1 = obj.ptonlink3(obj.r, obj.staticPoses.Case(ii).phi1); %Changed obj.r to obj.rB
                            obj.staticPoses.Case(ii).qB2 = obj.R12 * obj.ptonlink3(obj.r, obj.staticPoses.Case(ii).phi2); %Changed obj.r to obj.rB
                            obj.staticPoses.Case(ii).Rsb = obj.getRMatnumeric(obj.staticPoses.Case(ii).zB, obj.zs);
                            obj.staticPoses.Case(ii).psb = [0; 0; obj.r/sqrt(2)];
                            %obj.staticPoses.Case(ii).psb = [1; 1; obj.r/sqrt(2)];
                            obj.staticPoses.Case(ii).Tsb = [obj.staticPoses.Case(ii).Rsb, obj.staticPoses.Case(ii).psb; 0, 0, 0, 1];
                            obj.quasiStatic(obj.staticPoses.Case(ii).Tsb,obj.theta1,obj.theta2,obj.staticPoses.Case(ii).phi1,obj.staticPoses.Case(ii).phi2, obj.increment);
                            obj.iter = obj.iter + 1;
%                            staticPoses = obj.staticPoses;
                        end
                    end

%                     % Calculate qb1, qb2, and Tsb for two sets of phi1 and phi2 values
%                     obj.staticPoses.Case(ii).qB1 = obj.ptonlink3(obj.r, obj.staticPoses.Case(ii).phi1); %Changed obj.r to obj.rB
%                     obj.staticPoses.Case(ii).qB2 = obj.R12 * obj.ptonlink3(obj.r, obj.staticPoses.Case(ii).phi2); %Changed obj.r to obj.rB
%                     obj.staticPoses.Case(ii).Rsb = obj.getRMatnumeric(obj.staticPoses.Case(ii).zB, obj.zs);
%                     obj.staticPoses.Case(ii).psb = [0; 0; obj.r/sqrt(2)];
%                     obj.staticPoses.Case(ii).Tsb = [obj.staticPoses.Case(ii).Rsb, obj.staticPoses.Case(ii).psb; 0, 0, 0, 1];
 %                   nexttile()

%                    p1 = double(round(rad2deg(obj.staticPoses.Case(ii).phi1)));
%                    p2 = double(round(rad2deg(obj.staticPoses.Case(ii).phi2)));
    %                obj.drawTexplor(obj.staticPoses.Case(ii).Tsb,obj.theta1,obj.theta2,obj.staticPoses.Case(ii).phi1,obj.staticPoses.Case(ii).phi2, obj.staticPoses.Case(ii).zB, obj.staticPoses.Case(ii).Fr1, obj.staticPoses.Case(ii).Fr2, obj.iter);


                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %ADDING QUASI STUFF
%                     obj.quasiStatic(obj.staticPoses.Case(ii).Tsb,obj.theta1,obj.theta2,obj.staticPoses.Case(ii).phi1,obj.staticPoses.Case(ii).phi2, ii);
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                


%                    title({['State ',num2str(ii),', \phi_1=',num2str(p1) char(176),', \phi_2=',num2str(p2) char(176)]},'FontSize', 20);
%                     obj.iter = obj.iter + 1;
                
                end
                staticPoses = obj.staticPoses;
%             end
        end
        
        function drawTexplor(obj, Tsb, theta1, theta2, phi1, phi2, zB, Fr1, Fr2, iter)
            % Pre-transformation
            % Origin is (0,0,0) and the arc is in xy plane
            ptonlink3 = @(r,alpha) [r*cos(alpha);r*sin(alpha);zeros(size(alpha));ones(size(alpha))];
            plotarc = @(x,color) plot3(x(1,:),x(2,:),x(3,:),'Linewidth',5,'Color',color);
            plotmassp = @(x,color) plot3(x(1,:),x(2,:),x(3,:),'Marker','square','Linewidth',5,'MarkerSize',10,'Color',color);
            plotmassq = @(x,color) plot3(x(1,:),x(2,:),x(3,:),'Marker','o','Linewidth',5,'MarkerSize',5,'Color',color);
            
            % Calculate arcs, mass positions, and points of contact
            a1 = ptonlink3(obj.r,0:0.1:pi);
            a2 = obj.T12*a1;
            p1 = ptonlink3(obj.r,theta1);
            p2 = obj.T12*ptonlink3(obj.r,theta2);
            q1 = ptonlink3(obj.r,phi1);
            q2 = obj.T12*ptonlink3(obj.r,phi2);
            rCOM = Tsb*[0,0,0,1]';
            %rCOM = Tsb*(((p1*obj.m)+(p2*obj.m)+ (a1*obj.M)+(a2*obj.M))/((2*obj.m) + (2*obj.M)));
            
            % Plot the arcs
            plotarc(Tsb*a1,'r');
            hold on;
            plotarc(Tsb*a2,'r');
            
            % Plot the masses and the point of contacts
            plotmassp(Tsb*p1,'m');
            plotmassp(Tsb*p2,'m');
            plotmassq(Tsb*q1,'b');
            plotmassq(Tsb*q2,'b');
            plot3(rCOM(1),rCOM(2),rCOM(3),'Marker','x','Linewidth',2,'MarkerSize',10,'Color','k');
            axis equal;
            xground = [2 2 -2 -2];
            yground = [2 -2 -2 2];
            zground = [0 0 0 0];
            c = [0.8 0.7 0.8];
            fill3(xground,yground,zground,c,'FaceAlpha',0.8);

            xlim([-2 2]);
            ylim([-2 2]);
            zlim([0 2]);

            annstr = ['\theta_1=',num2str(rad2deg(obj.theta1)) char(176),', \theta_2=',num2str(rad2deg(obj.theta2)) char(176)];
            %annstr = 'test';
            pos = [0.35 0.3 0.3 0.3];
            ann = annotation('textbox',pos,'String',annstr,'FitBoxToText','on', 'HorizontalAlignment',"center", 'FontSize', 20);
            %ann.HorizontalAlignment = 'center';
            ann.BackgroundColor = [0.8 0.8 0.8]; % make the box opaque with some color


            % if (Fr1 > 0 && Fr2 > 0) %Check if both forces are positive
            %     %annstr = sprintf(' ');
            %     if (iter == 1)
            %         annpos = [0.05 0.9 0.05 0.07]; % annotation position in figure coordinates
            %     elseif (iter == 2)
            %         annpos = [0.9 0.9 0.05 0.07]; % annotation position in figure coordinates
            %     elseif (iter == 3)
            %         annpos = [0.05 0.05 0.05 0.07]; % annotation position in figure coordinates
            %     elseif (iter == 4)
            %         annpos = [0.9 0.05 0.05 0.07]; % annotation position in figure coordinates
            %     end 
            %     annotation('ellipse',annpos,'Color','green','FaceColor','green');
            %     %ann.HorizontalAlignment = 'center';
            %     %ann.BackgroundColor = [0 1 0]; % make the box opaque with some color
            % else
            %     %annstr = sprintf(' ');
            %     if (iter == 1)
            %         annpos = [0.05 0.9 0.05 0.07]; % annotation position in figure coordinates
            %     elseif (iter == 2)
            %         annpos = [0.9 0.9 0.05 0.07]; % annotation position in figure coordinates
            %     elseif (iter == 3)
            %         annpos = [0.05 0.05 0.05 0.07]; % annotation position in figure coordinates
            %     elseif (iter == 4)
            %         annpos = [0.9 0.05 0.05 0.07]; % annotation position in figure coordinates
            %     end 
            %     annotation('ellipse',annpos,'Color','red','FaceColor','red');
            %     %ann.HorizontalAlignment = 'center';
            %     %ann.BackgroundColor = [1 0 0]; % make the box opaque with some color
            % end
        end
        
        %ADDED FOR QUASISTATIC STUFF
        function quasiStatic(obj, Tsb, theta1, theta2, phi1, phi2, ii)
            % Pre-transformation
            % Origin is (0,0,0) and the arc is in xy plane
            
            ptonlink3 = @(r,alpha) [r*cos(alpha);r*sin(alpha);zeros(size(alpha));ones(size(alpha))];
            %ptonlinkTangent3 = @(r,alpha) [r*-sin(alpha); r*cos(alpha); 0; 0];



            % plotarc1 = @(x,color) plot3(x(1,:)+(obj.iter/20),x(2,:),x(3,:),'Linewidth',3,'Color',color);
            % plotarc2 = @(x,color) plot3(x(1,:)+(obj.iter/20),x(2,:),x(3,:),'Linewidth',3,'Color',color);
            % plotmassp = @(x,color) plot3(x(1,:)+(obj.iter/20),x(2,:),x(3,:),'Marker','o','Linewidth',5,'MarkerSize',10,'Color',color);
            % plotmassq = @(x,color) plot3(x(1,:)+(obj.iter/20),x(2,:),x(3,:),'Marker','o','Linewidth',5,'MarkerSize',5,'Color',color);
            % plotCoM = @(x,color) plot3(x(1)+(obj.iter/20),x(2),x(3),'Marker','x','Linewidth',5,'MarkerSize',20,'Color',color);



            plotarc1 = @(x,color) plot3(x(1,:),x(2,:),x(3,:),'Linewidth',3,'Color',color);
            plotarc2 = @(x,color) plot3(x(1,:),x(2,:),x(3,:),'Linewidth',3,'Color',color);
            plotmassp = @(x,color) plot3(x(1,:),x(2,:),x(3,:),'Marker','o','Linewidth',5,'MarkerSize',10,'Color',color);
            plotmassq = @(x,color) plot3(x(1,:),x(2,:),x(3,:),'Marker','o','Linewidth',5,'MarkerSize',5,'Color',color);
            plotCoM = @(x,color) plot3(x(1),x(2),x(3),'Marker','x','Linewidth',5,'MarkerSize',20,'Color',color);
            
%            obj.increment = obj.increment + 1;


            % Calculate arcs, mass positions, and points of contact
            a1 = ptonlink3(obj.r,0:0.1:pi);
            a2 = obj.T12*a1;
            p1 = ptonlink3(obj.r,theta1);
            p2 = obj.T12*ptonlink3(obj.r,theta2);
            q1 = ptonlink3(obj.r,phi1);
            q2 = obj.T12*ptonlink3(obj.r,phi2);
            

            %Trying out new rCOM
            %rCOM = Tsb*[0,0,0,1]';
            rCOM = Tsb*(((p1*obj.m)+(p2*obj.m)+ (a1*obj.M)+(a2*obj.M))/((2*obj.m) + (2*obj.M)));


            if ii == 1
                color = 'm';
            end
            if ii == 2
                color = 'b';
            end
            if ii == 3
                color = 'g';
            end
            if ii == 4
                color = 'r';
            end
            % Plot the arcs
            plotarc1(Tsb*a1,color);
            hold on;
            %plotarc2(Tsb*a2,[0 1 0]);
            plotarc2(Tsb*a2,color);
            hold on;
            % Plot the masses and the point of contacts
            plotmassp(Tsb*p1,'m');
            plotmassp(Tsb*p2,'m');
            plotmassq(Tsb*q1,'b');
            plotmassq(Tsb*q2,'b');
            plotCoM(rCOM,'k');
            axis equal;
            %xground = [2 2 -2 -2];
            %yground = [2 -2 -2 2];
            %c = [0.8 0.7 0.8];
            %fill(xground,yground,c,'FaceAlpha',0.8);
            hold on;
        end
        
    end
end