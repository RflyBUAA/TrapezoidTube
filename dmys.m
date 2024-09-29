function u=dmys(x,rs)%±¥ºÍº¯Êý

x2 =  1 + 1/tan(67.5/180*pi)*rs;
x1 = x2 - sin(45/180*pi)*rs;
if x<=x1
    u = 1;
elseif x1<=x  & x<=x2
    u = (x2-x)/sqrt(rs^2-(x-x2)^2)/2;
else
    u = 0;
end