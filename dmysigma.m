function u=dmysigma(x,d1,d2)

if x<=d1
    u = 0;
elseif d1<=x  & x<=d2
    A = -2/((d1-d2)^3); B = 3*(d1+d2)/((d1-d2)^3); C = -6*d1*d2/((d1-d2)^3);
    u = 3*A*x^2 + 2*B*x + C;
else
    u = 0;
end