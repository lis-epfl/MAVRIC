function [x_kk,P_kk] = ekf_update_mag(x_kk1,P_kk1,R_mag,z_mag,type)

type = 1;

h_kk1 = zeros(3,1);

mag_global = [cos(deg2rad(63)) 0 sin(deg2rad(63))];

if type == 1
    h_kk1(1) = (1-2*(x_kk1(6)^2+x_kk1(7)^2))*mag_global(1)...
        + 2*(x_kk1(5)*x_kk1(7) - x_kk1(4)*x_kk1(6))*mag_global(3);
    h_kk1(2) = 2*(x_kk1(5)*x_kk1(6) - x_kk1(4)*x_kk1(7))*mag_global(1)...
            + 2*(x_kk1(6)*x_kk1(7) + x_kk1(4)*x_kk1(5))*mag_global(3);
	h_kk1(3) = 2*(x_kk1(5)*x_kk1(7) - x_kk1(4)*x_kk1(6))*mag_global(1)...
            + (1-2*(x_kk1(5)^2+x_kk1(6)^2))*mag_global(3);
else
    h_kk1(1) = (x_kk1(4)^2+x_kk1(5)^2-x_kk1(6)^2-x_kk1(7)^2)*mag_global(1)...
        + 2*(x_kk1(5)*x_kk1(7) - x_kk1(4)*x_kk1(6))*mag_global(3);
    h_kk1(2) = 2*(x_kk1(5)*x_kk1(6) - x_kk1(4)*x_kk1(7))*mag_global(1)...
            + 2*(x_kk1(6)*x_kk1(7) + x_kk1(4)*x_kk1(5))*mag_global(3);
    h_kk1(3) = 2*(x_kk1(5)*x_kk1(7) - x_kk1(4)*x_kk1(6))*mag_global(1)...
            + (x_kk1(4)^2-x_kk1(5)^2-x_kk1(6)^2+x_kk1(7)^2)*mag_global(3);
end

H_k = zeros(3,7);

if type == 1
    H_k(1,4) = -2*x_kk1(6)*mag_global(3);
    H_k(1,5) = 2*x_kk1(7)*mag_global(3);
    H_k(1,6) = -4*x_kk1(6)*mag_global(1) - 2*x_kk1(4)*mag_global(3);
    H_k(1,7) = -4*x_kk1(7)*mag_global(1) + 2*x_kk1(5)*mag_global(3);
    
    H_k(2,4) = -2*x_kk1(7)*mag_global(1) + 2*x_kk1(5)*mag_global(3);
    H_k(2,5) = 2*x_kk1(6)*mag_global(1) + 2*x_kk1(4)*mag_global(3);
    H_k(2,6) = 2*x_kk1(5)*mag_global(1) + 2*x_kk1(7)*mag_global(3);
    H_k(2,7) = -2*x_kk1(4)*mag_global(1) + 2*x_kk1(6)*mag_global(3);
    
    H_k(3,4) = -2*x_kk1(6)*mag_global(1);
    H_k(3,5) = 2*x_kk1(7)*mag_global(1)-4*x_kk1(5)*mag_global(3);
    H_k(3,6) = -2*x_kk1(4)*mag_global(1)-4*x_kk1(6)*mag_global(3);
    H_k(3,7) = 2*x_kk1(5)*mag_global(1);
else
    H_k(1,4) = 2*x_kk1(4)*mag_global(1)-2*x_kk1(6)*mag_global(3);
    H_k(1,5) = 2*x_kk1(5)*mag_global(1)+2*x_kk1(7)*mag_global(3);
    H_k(1,6) = -2*x_kk1(6)*mag_global(1) - 2*x_kk1(4)*mag_global(3);
    H_k(1,7) = -2*x_kk1(7)*mag_global(1) + 2*x_kk1(5)*mag_global(3);
    
    H_k(2,4) = -2*x_kk1(7)*mag_global(1) + 2*x_kk1(5)*mag_global(3);
    H_k(2,5) = 2*x_kk1(6)*mag_global(1) + 2*x_kk1(4)*mag_global(3);
    H_k(2,6) = 2*x_kk1(5)*mag_global(1) + 2*x_kk1(7)*mag_global(3);
    H_k(2,7) = -2*x_kk1(4)*mag_global(1) + 2*x_kk1(6)*mag_global(3);
    
    H_k(3,4) = -2*x_kk1(6)*mag_global(1)+2*x_kk1(4)*mag_global(3);
    H_k(3,5) = 2*x_kk1(7)*mag_global(1)-2*x_kk1(5)*mag_global(3);
    H_k(3,6) = -2*x_kk1(4)*mag_global(1)-2*x_kk1(6)*mag_global(3);
    H_k(3,7) = 2*x_kk1(5)*mag_global(1)+2*x_kk1(7)*mag_global(3);
end
y_k = z_mag - h_kk1;

S_k = H_k * P_kk1 * H_k' + R_mag;

K_mag = P_kk1 * H_k' / S_k;

x_kk = x_kk1 + K_mag*y_k;

normX = norm(x_kk(4:7),2);
x_kk(4:7) = x_kk(4:7) / normX;

P_kk = (eye(7) - K_mag*H_k)*P_kk1;
