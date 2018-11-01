function L = L(x)

% Assumes single integrator model so that we have 3 states for a single
% agent in the X vector and z is same and constant for all the quadrotors
% Assuming 2 quadrotors and 1 ground robot

theta_1 = 60*pi/180;
theta_2 = 90*pi/180;
R = ( x(6)*sin(theta_1/2) );
rho = ( x(6)*sin(theta_2/2) );
e = exp(1);

%a = -5*(R-rho);

L = cvx(zeros(3));

        L(1,2) = -min(1,pow_p(e, R - r(1,2,x)));
        L(1,3) = -min(1,pow_p(e, R - r(1,3,x)));
        L(1,1) = -L(1,2) - L(1,3);
        
        L(2,1) = L(1,2);
        L(2,3) = 0;
        L(2,2) = -L(2,1) - L(2,3);
         
        L(3,1) = L(1,3);
        L(3,2) = L(2,3);
        L(3,3) = -L(3,1) - L(3,2);
         

% L(1,2) = min(1,exp( R - r(1,2,x) )); %min(1,1*inv_pos(r(1,2,x))-1);
% L(1,3) = min(1,exp( R - r(1,3,x) )); %min(1,exp(5*(r(1,3,x) - R) / (R - rho)));
% L(1,2) = -L(1,2); L(1,3) = -L(1,3);
% 
% L(1,1) = -L(1,2) - L(1,3);
% 
% L(2,1) = L(1,2);
% L(2,3) = 0;
% L(2,2) = -L(2,1) - L(2,3);
% 
% L(3,1) = L(1,3);
% L(3,2) = L(2,3);
% L(3,3) = -L(3,1) - L(3,2);
end
