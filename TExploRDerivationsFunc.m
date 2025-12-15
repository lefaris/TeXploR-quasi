% Agile Robotics Laboratory at UA
% TExploR Project
% Date: 02/26/2024
% 
% Calculates phi_i, zB, and Fri values for a given body wrench dependent
% on which of the four states TExploR is in.  Using this code allows us to
% make modifications to the initial zB and body wrench equations and the
% appropriate phi_i, zB, and Fri values will automatically be calculated.
% In other words, we do not hard code the equations for phi_i, zB, and Fri
% values; they are solved for based off the initialy equations stated.
% 
% Usage:
% Run the TExploRStaticsDerivations.m function by calling
% TExploRStaticDerivations(0,0).  That code will call this class to 
% calculate phi_i, zB, and Fri values. 

classdef TExploRDerivationsFunc
methods (Static)
function cls = setup
syms t real positive
syms theta phi M [2 1] real
syms r real
syms mg Mg Fr1 Fr2 real

% Kinematics
ptonlink = @(r,alpha) r*[cos(alpha),sin(alpha),0,1/r]';
tangentonlink = @(r,alpha) r*[-sin(alpha),cos(alpha),0,0]';
binormalonlink = @(r,alpha) -r*[cos(alpha),sin(alpha),0,0]';
cls.pB1 = ptonlink(r,theta(1));
cls.qB1 = ptonlink(r,phi(1));
cls.tB1 = tangentonlink(r,phi(1));
cls.bB1 = binormalonlink(r,phi(1));
cls.rB1 = 0.15*[0,1,0,1/0.15]';

cls.T12 = [0,0,1,0;
    0,-1,0,0;
    1,0,0,0;
    0,0,0,1];
cls.rB2 = cls.T12*cls.rB1;
cls.pB2 = cls.T12*ptonlink(r,theta(2));
cls.qB2 = cls.T12*ptonlink(r,phi(2));
cls.tB2 = cls.T12*tangentonlink(r,phi(2));
cls.bB2 = cls.T12*binormalonlink(r,phi(2));
end

% Holonomic Constraint
% Case 2: t1 exists but t2 doesn't

% Case 2a where phi2 = 0
function [phi2Areturn, zB1aReturn, Fr2a1, Fr2a2] = c2a(m,M,g,t1,t2)
cls = TExploRDerivationsFunc.setup;
syms t real positive
syms theta phi [2 1] real
syms r real
syms mg Mg Fr1 Fr2 real
phi2a = 0;
zB1a = simplify(LieGroup.vec2so3(cls.tB1(1:3))*subs((cls.qB1(1:3)-cls.qB2(1:3)),phi2,phi2a))/(sqrt(2)*r^2);
Wrench2a = simplify([2*mg+2*Mg-Fr1-Fr2;(2*mg*LieGroup.vec2so3(cls.rB1(1:3)+cls.rB2(1:3))+2*Mg*LieGroup.vec2so3(cls.pB1(1:3)+cls.pB2(1:3))-...
    Fr1*LieGroup.vec2so3(cls.qB1(1:3))-Fr2*LieGroup.vec2so3(subs(cls.qB2(1:3),phi2,phi2a)))*zB1a]);
Fcoeffs2a = [coeffs(Wrench2a(1),{Fr1,Fr2});coeffs(Wrench2a(2),{Fr1,Fr2});coeffs(Wrench2a(3),{Fr1,Fr2})];
big2aA = Fcoeffs2a(:,2:3); smallB2a = Fcoeffs2a(:,1);
Frs2a = simplify(pinv(big2aA)*smallB2a);
Wrench2aa = simplify(subs(Wrench2a,{Fr1,Fr2},{Frs2a(1),Frs2a(2)})/(Mg*r));
eqn = simplify(Wrench2aa(3));
p2a = solve(eqn,phi1,'Real',true);

% To avoid division by 0 error
if (t1 > 1.5 && t1 < 1.6)
    phi2Areturntemp = 0;
else
    phi2Areturntemp = atan((sin(t1) - sin(t2))/cos(t1));
end

%Added for edge cases of (0,0) & (180,180) 
if ((t1 == 0) && (t2 == 0))
    phi2Areturn = -phi2a;
elseif ((t1 == pi) && (t2 == pi))
    phi2Areturn = phi2a + pi;
else
    phi2Areturn = (1 - sign(phi2Areturntemp)) * pi / 2 + phi2Areturntemp;
end

zB1aReturn = subs(zB1a,phi1,phi2Areturn);
Frs2a = subs(Frs2a,{Mg,mg,phi1,theta1,theta2},{M*-g,m*-g,phi2Areturn,t1,t2});
Fr2a1 = Frs2a(1);
Fr2a2 = Frs2a(2);
end

% Case 2b where phi2 = pi
function [phi2Breturn, zB1bReturn, Fr2b1, Fr2b2] = c2b(m,M,g,t1,t2)
cls = TExploRDerivationsFunc.setup;
syms t real positive
syms theta phi [2 1] real
syms r real
syms mg Mg Fr1 Fr2 real
phi2b = pi;
zB1b = simplify(LieGroup.vec2so3(cls.tB1(1:3))*subs((cls.qB1(1:3)-cls.qB2(1:3)),phi2,phi2b)/(sqrt(2)*-r^2));
Wrench2b = simplify([2*mg+2*Mg-Fr1-Fr2;(2*mg*LieGroup.vec2so3(cls.rB1(1:3)+cls.rB2(1:3))+2*Mg*LieGroup.vec2so3(cls.pB1(1:3)+cls.pB2(1:3))-...
    Fr1*LieGroup.vec2so3(cls.qB1(1:3))-Fr2*LieGroup.vec2so3(subs(cls.qB2(1:3),phi2,phi2b)))*zB1b]);
Fcoeffs2b = [coeffs(Wrench2b(1),{Fr1,Fr2});coeffs(Wrench2b(2),{Fr1,Fr2});coeffs(Wrench2b(3),{Fr1,Fr2})];
big2bA = Fcoeffs2b(:,2:3); smallB2b = Fcoeffs2b(:,1);
Frs2b = simplify(pinv(big2bA)*smallB2b);
Wrench2bb = simplify(subs(Wrench2b,{Fr1,Fr2},{Frs2b(1),Frs2b(2)})/(Mg*r));
eqn = simplify(Wrench2bb(3));
p2b = solve(eqn,phi1,'Real',true);

% To avoid division by 0 error
if (t1 > 1.5 && t1 < 1.6)
    phi2Breturntemp = 0;
else
    phi2Breturntemp = atan((sin(t1) - sin(t2))/cos(t1));
end

%Added for edge cases of (0,0) & (180,180) 
if ((t1 == 0) && (t2 == 0))
    phi2Breturn = phi2b - pi;
elseif ((t1 == pi) && (t2 == pi))
    phi2Breturn = phi2b;
else
    phi2Breturn = (1 - sign(phi2Breturntemp)) * pi / 2 + phi2Breturntemp;
end

zB1bReturn = subs(zB1b,phi1,phi2Breturn);
Frs2b = subs(Frs2b,{Mg,mg,phi1,theta1,theta2},{M*-g,m*-g,phi2Breturn,t1,t2});
Fr2b1 = Frs2b(1);
Fr2b2 = Frs2b(2);
end

% Case 3: t2 exists but t1 doesn't

% Case 3a where phi1 = 0
function [phi3Areturn, zB2aReturn, Fr3a1, Fr3a2] = c3a(m,M,g,t1,t2)
cls = TExploRDerivationsFunc.setup;
syms t real positive
syms theta phi [2 1] real
syms r real
syms mg Mg Fr1 Fr2 real
phi1a=0;
zB2a = simplify(LieGroup.vec2so3(cls.tB2(1:3))*subs((cls.qB1(1:3)-cls.qB2(1:3)),phi1,phi1a)/(-r^2));
Wrench3a = simplify([2*mg+2*Mg-Fr1-Fr2;(2*mg*LieGroup.vec2so3(cls.rB1(1:3)+cls.rB2(1:3))+2*Mg*LieGroup.vec2so3(cls.pB1(1:3)+cls.pB2(1:3))-...
    Fr1*LieGroup.vec2so3(subs(cls.qB1(1:3),phi1,phi1a))-Fr2*LieGroup.vec2so3(cls.qB2(1:3)))*zB2a]);
Fcoeffs3a = [coeffs(Wrench3a(1),{Fr1,Fr2});coeffs(Wrench3a(3),{Fr1,Fr2});coeffs(Wrench3a(4),{Fr1,Fr2})];
big3aA = Fcoeffs3a(:,2:3); smallB3a = Fcoeffs3a(:,1);
Frs3a = simplify(pinv(big3aA)*smallB3a);
Wrench3aa = simplify(subs(Wrench3a,{Fr1,Fr2},{Frs3a(1),Frs3a(2)})/(2*Mg*r^3));
eqn = simplify(Wrench3aa(3));
p3a = solve(eqn,phi2,'Real',true);

% To avoid division by 0 error
if (t2 > 1.5 && t2 < 1.6)
    phi3Areturntemp = 0;
else
    phi3Areturntemp = -atan((sin(t1) - sin(t2))/cos(t2));
end

%Added for edge cases of (0,0) & (180,180) 
if ((t1 == 0) && (t2 == 0))
    phi3Areturn = -phi1a;
elseif ((t1 == pi) && (t2 == pi))
    phi3Areturn = phi1a + pi;
else
    phi3Areturn = (1 - sign(phi3Areturntemp)) * pi / 2 + phi3Areturntemp;
end

zB2aReturn = subs(zB2a,phi2,phi3Areturn);
Frs3a = subs(Frs3a,{Mg,mg,phi2,theta1,theta2},{M*-g,m*-g,phi3Areturn,t1,t2});
Fr3a1 = Frs3a(1);
Fr3a2 = Frs3a(2);
end

% Case 3b where phi1 = pi
function [phi3Breturn, zB2bReturn, Fr3b1, Fr3b2] = c3b(m,M,g,t1,t2)
cls = TExploRDerivationsFunc.setup;
syms t real positive
syms theta phi [2 1] real
syms r real
syms mg Mg Fr1 Fr2 real
phi1b = pi;
zB2b = simplify(LieGroup.vec2so3(cls.tB2(1:3))*subs((cls.qB1(1:3)-cls.qB2(1:3)),phi1,phi1b)/(r^2));
Wrench3b = simplify([2*mg+2*Mg-Fr1-Fr2;(2*mg*LieGroup.vec2so3(cls.rB1(1:3)+cls.rB2(1:3))+2*Mg*LieGroup.vec2so3(cls.pB1(1:3)+cls.pB2(1:3))-...
    Fr1*LieGroup.vec2so3(subs(cls.qB1(1:3),phi1,phi1b))-Fr2*LieGroup.vec2so3(cls.qB2(1:3)))*zB2b]);
Fcoeffs3b = [coeffs(Wrench3b(1),{Fr1,Fr2});coeffs(Wrench3b(3),{Fr1,Fr2});coeffs(Wrench3b(4),{Fr1,Fr2})];
big3bA = Fcoeffs3b(:,2:3); smallB3b = Fcoeffs3b(:,1);
Frs3b = simplify(pinv(big3bA)*smallB3b);
Wrench3bb = simplify(subs(Wrench3b,{Fr1,Fr2},{Frs3b(1),Frs3b(2)})/(Mg*r));
eqn = simplify(Wrench3bb(3));
p3b = solve(eqn,phi2,'Real',true);

% To avoid division by 0 error
if (t2 == (pi/2))
    phi3Breturntemp = 0;
else
    phi3Breturntemp = -atan((sin(t1) - sin(t2))/cos(t2));
end

%Added for edge cases of (0,0) & (180,180) 
if ((t1 == 0) && (t2 == 0))
    phi3Breturn = phi1b - pi;
elseif ((t1 == pi) && (t2 == pi))
    phi3Breturn = phi1b;
else
    phi3Breturn = (1 - sign(phi3Breturntemp)) * pi / 2 + phi3Breturntemp;
end
zB2bReturn = subs(zB2b,phi2,phi3Breturn);
Frs3b = subs(Frs3b,{Mg,mg,phi2,theta1,theta2},{M*-g,m*-g,phi3Breturn,t1,t2});
Fr3b1 = Frs3b(1);
Fr3b2 = Frs3b(2);
end

end
end