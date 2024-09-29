function u=dmysigma2(x,d1,d2,em)
A =-(d2*em - d1*em + 2)/(d1 - d2)^3;
B =(- em*d1^2 - em*d1*d2 + 3*d1 + 2*em*d2^2 + 3*d2)/(d1 - d2)^3;
C =-(d2*(- 2*em*d1^2 + em*d1*d2 + 6*d1 + em*d2^2))/(d1 - d2)^3;
D =(d2*(- em*d1^2*d2 + em*d1*d2^2 + 3*d1*d2 - d2^2))/((d1 - d2)*(d1^2 - 2*d1*d2 + d2^2));
if x<=d1
    u = 1;
elseif d1<=x  && x<=d2
    u = A*x^3 + B*x^2 + C*x + D ;
else
    u = 0;
end