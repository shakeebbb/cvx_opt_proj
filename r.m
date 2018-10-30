function r = r(i,j,x)
r = norm([x(3*i - 2) - x(3*j - 2),x(3*i - 1) - x(3*j - 1)]);
%r = (x(3*i - 2) - x(3*j - 2))^2 + (x(3*i - 1) - x(3*j - 1))^2; % + (X(3*i + 2) - X(3*j + 2))^2;
end