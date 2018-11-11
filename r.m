function r = r(i,j,x,n)
% a = x(n*i - n + 1)
% b = x(n*j - n + 1)
% c = x(n*i - n + 2)
% d = x(n*i - n + 2)

r = norm([x(n*i - n + 1) - x(n*j - n + 1),x(n*i - n + 2) - x(n*j - n + 2)]);
%r = (x(3*i - 2) - x(3*j - 2))^2 + (x(3*i - 1) - x(3*j - 1))^2; % + (X(3*i + 2) - X(3*j + 2))^2;
end