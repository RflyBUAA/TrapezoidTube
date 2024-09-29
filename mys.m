function u=mys(x,rs)

x2 =  1 + 1/tan(67.5/180*pi)*rs;
x1 = x2 - sin(45/180*pi)*rs;
if x<=x1
    u = x;
elseif x1<=x  & x<=x2
    u = (1-rs)+sqrt(rs^2-(x-x2)^2);
else
    u = 1;
end