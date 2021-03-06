function [dq] = mimo_pd_zd(t, q, u)
    q = q(:);
    z = q(1:2);
    x = q(3:end);
    x1 = x([1 4]);
    x2 = x([2 5]);
    x3 = x([3 6]);
    
    d = [15*(t > 4.5 && t < 5.5);
         12*(t > 5 && t < 6)];

    f = [0.7.*x2(2)*x3(1) + 0.3*(z(2) + 3/4)^2;
         0.3.*x2(1)*x3(2) + 0.7*(z(1) + 1)^2];
     
%     G11 = 1.2 + cos(x1(1)).*sin(x1(2));
%     G12 = 0.3*cos(x1(1)*x1(2));
%     G22 = 1.7 + sin(x1(1)).*cos(x1(2));

    G11 = 1.2 + cos(x3(1)).*sin(x3(2));
    G12 = 0.3*cos(x2(1)*x2(2));
    G22 = 1.7 + sin(x3(1)).*cos(x3(2));


    G = [G11 G12;
         G12 G22];

    Az = -[1/2, 1/3;
           2/3, 3/4];
       
    dz = Az*z + x2 + [0.3*sin(t); 0.2*cos(t)];
    dx = zeros(6, 1);
    dx([1 4]) = x2;
    dx([2 5]) = x3;
    dx([3 6]) = f + G*u + d;
    dq = [dz; dx];
end

