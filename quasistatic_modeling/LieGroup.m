classdef LieGroup
    properties (Access=public)
        Screw; 
        Magnitude;
        Twist; % [omega',v']'
        R; p;
    end
    properties (Access=private)
        zeroThreshold;
    end
    methods
        function obj = LieGroup(R,p)
            obj.R = R;
            obj.p = p;
        end
%         Threshold that declares if a number is assumed to be less that
%         zero
        function setzeroThreshold(obj,val)
            obj.zeroThreshold = val;
        end
    end
    methods (Static)
        % Function converts a so3 to a 3x1 vector
        % Input:   skew symmetric matrix (3x3)
        % Output:  vector 'x' (3x1)
        % Example: [x] = so32vec(X)
        % 
        function [x] = so32vec(X)
            x= [X(3,2);X(1,3);X(2,1)];
        end
        % so(3) - Skew symmetric matrix
        % Input:    vector 'x' (3x1)
        % Output:   skew symmetric matrix (3x3)
        % Example: [X] =vec2so3(x)
        % 
        function [X] =vec2so3(x)
         X = [0, -x(3),x(2);x(3), 0, -x(1);-x(2), x(1),0];
        end
        % se(3) 
        % Input:    vector 'x' (6x1)
        % Output:   se3 matrix (4x4)
        % Example: [X] =vec2se3(x)
        % 
        function [X] =vec2se3(x)
             X = [LieGroup.vec2so3(x(1:3)),x(4:6);zeros(1,4)];
        end
        % Function converts a se3 to a 6x1 vector
        % Input:   se3 matrix (4x4)
        % Output:  vector 'x' (6x1)
        % Example: [x] = se32vec(X)
        % 
        function [x] = se32vec(X)
            x= [LieGroup.so32vec(X(1:3,1:3));X(1:3,4)];
        end
        
        % The function obtains the rotation matrix for a rotation about axis
        % 'omega' by angle 'theta'
        % 
        % [R] = MatExponential3(omega,theta)
        % Input: 
        % 'omega': 3x1 column vector corresponding to the axis of rotation
        % 'theta': 1x1 scalar denoting the angle of rotation in radians
        % 
        % Output:
        % R : 3x3 rotation matrix that is an element of SO(3)
        % 
        % Example:
        % R = MatExponential([1,0,0]',pi/4) 
        % { Rotation of 45 degrees about the x-axis }
        % R = MatExponential([0,1,0]',-pi/3)
        % { Rotation of -60 degrees about y-axis }
        % 
        % Author: vvikas@ua.edu 
        function [R] = MatExponential3(omega,theta)
            zeroThreshold = 1e-12;
            if and(norm(omega) < zeroThreshold,isnumeric(omega))
                R = eye(3);
            else
                R = eye(3) + sin(theta)*LieGroup.vec2so3(omega)+...
                    (1-cos(theta))*(LieGroup.vec2so3(omega)^2);
            end
        end
        % Calculates the Transformation Matrix given a twist - screw (omega, v) and
        % its magnitude (theta)
        % 
        % Input: 
        % omega: 3x1 vector (rotational component of the screw) - unit vector
        % theta: 1x1 scalar magrinude of the screw
        % v: 3x1 vector (translational velocity component of the screw)
        % 
        % Output:
        % T: 4x4 transformation matrix
        % 
        % [T] = TMatExponential3(omega,theta,v)
        %
        % vvikas@ua.edu
        % 
        function [T] = TMatExponential3(omega,theta,v) 
            zeroThreshold = 1e-12;
            if and(norm(omega) < zeroThreshold,isnumeric(omega))
                R = eye(3);
                p=v*theta;
            else
                R = LieGroup.MatExponential3(omega,theta);
                p = (eye(3)-R)*LieGroup.vec2so3(omega)*v + omega*omega'*v*theta;
            end
            T = [R,p;zeros(1,3),1];
        end
        % Calculates the inverse of a transformation matrix
        % 
        % Input:
        % T: 4x4 transformation matrix
        % 
        % Output: 
        % Matrix inverse: 4x4 transformation matrix
        % 
        % vvikas@ua.edu
        % 
        function [Tinv] = TInv(T)
            R = T(1:3,1:3); p = T(1:3,4);
            Tinv = [R', -R'*p;zeros(1,3), 1];
        end
        % % The function obtains the equivalent axis 'omegaHat' and angle 'theta' of
        % rotation given a rotation matrix R
        % 
        % [omegaHat, theta] = EquivalentAxis3(R)
        % Input: 
        % R : 3x3 rotation matrix that is an element of SO(3)
        % 
        % Output:
        % 'omegaHat': 3x1 unit column vector corresponding to the axis of rotation
        % 'theta': 1x1 scalar denoting the angle of rotation in radians
        % 
        % Author: vvikas@ua.edu 
        function [omegaHat, theta] = EquivalentAxis3(R)
            zeroThreshold = 1e-12;
            if and(abs(trace(R)-3) < zeroThreshold, isnumeric(R)) 
                disp('No rotation, equivalent axis undefined');
                theta = 0;
                omegaHat =NaN;
            elseif and((abs(trace(R)+1)<zeroThreshold),isnumeric(R))
                theta = pi;
                if ((1+R(1,1))>zeroThreshold)
                    omegaHat = [(1+R(1,1));R(2,1);R(3,1)]/sqrt(2*(1+R(1,1)));
                elseif ((1+R(2,2))>zeroThreshold)
                    omegaHat = [R(1,2);(1+R(2,2));R(3,2)]/sqrt(2*(1+R(2,2)));
                else
                    omegaHat = [R(1,3);R(2,3);(1+R(3,3))]/sqrt(2*(1+R(3,3)));
                end
            else
                theta = acos((trace(R)-1)/2);
                omegaHat = 1/(2*sin(theta))*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
            end
        end        
        % Calculates the Adjoint given a transformation matrix
        % 
        % Input:
        % T: 4x4 transformation matrix
        % 
        % Output: 
        % Adjoint: 6x6 Adjoint matrix
        % 
        % vvikas@ua.edu
        % 
        function [Adj] = Adjoint3(T)
            R = T(1:3,1:3); p = T(1:3,4);
            Adj = [R, zeros(3);vec2so3(p)*R, R];
        end
        % Calculates the small adjoint given a body twist for the
        % manipulator special case
        % 
        % Input:
        % V: 1x6 body twist
        % 
        % Output: 
        % Adjoint: 6x6 adjoint matrix
        % 
        % lefaris@crimson.ua.edu
        % 
        function [Adj] = Adjoint6(V)
            omega = V(1:3); v = V(4:6);
            Adj = [vec2so3(omega), vec2so3(v); zeros(3) vec2so3(omega)];
        end

        % Calculates the screw (omega, v) and its magnitude (theta) given a
        % transformation matrix
        % 
        % Input:
        % T: 4x4 transformation matrix
        % 
        % Output: 
        % omega: 3x1 vector (rotational component of the screw) - unit vector
        % theta: 1x1 scalar magrinude of the screw
        % v: 3x1 vector (translational velocity component of the screw)
        % 
        % vvikas@ua.edu
        % 
        function [omega,theta,v] = EquivalentScrew3(T)
            R = T(1:3,1:3); p = T(1:3,4);
            if trace(R)==3 % No rotation
                omega =[0,0,0]';
                theta = norm(p);
                v = p/theta;
            else
            [omega,theta] = EquivalentAxis3(R);
            omegabracket = vec2so3(omega);
            Ginv =(eye(3)/theta - (omegabracket/2)+ (1/theta-(cot(theta/2)/2))*omegabracket^2);
            v = Ginv*p;
            end
        end
        %
        % This function outputs the geometric quantities related to the
        % screw - namely, the pitch 'h' and point of rotation 'rho'. It is
        % assumed that this point 'rho' is normal to the axis of rotation.
        % Input: S - screw, 6x1 vector (need not be normalized)
        function [geoRep] = geometricRepresentation(S)
            omega = S(1:3);
            v = S(4:6);
            zeroThreshold = 1e-12;
            if and(abs(omega)<zeroThreshold,isnumeric(S))
                geoRep.h = Inf;
                geoRep.rho = zeros(3,1);
                geoRep.mag = norm(v);
            else
                geoRep.h = omega'*v/(omega'*omega);
                geoRep.rho = LieGroup.vec2so3(omega)*v/(omega'*omega);
                geoRep.mag = norm(omega);
            end
        end
    end       
end