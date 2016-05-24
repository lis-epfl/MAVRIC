function [x_kk,P_kk] = ekf_update_acc(x_kk1,P_kk1,R_acc,z_acc,type)

h_kk1 = zeros(3,1);

acc_z_global = -1;

if type == 1
    h_kk1(1) = 2*(x_kk1(5)*x_kk1(7) - x_kk1(4)*x_kk1(6)) * acc_z_global;
    h_kk1(2) = 2*(x_kk1(6)*x_kk1(7) + x_kk1(4)*x_kk1(5)) * acc_z_global;
    h_kk1(3) = (1 - 2*(x_kk1(5)^2 + x_kk1(6)^2)) * acc_z_global;
else
    h_kk1(1) = 2*(x_kk1(5)*x_kk1(7) - x_kk1(4)*x_kk1(6)) * acc_z_global;
    h_kk1(2) = 2*(x_kk1(6)*x_kk1(7) + x_kk1(4)*x_kk1(5)) * acc_z_global;
    h_kk1(3) = (x_kk1(4)^2 - x_kk1(5)^2 - x_kk1(6)^2 + x_kk1(7)^2) * acc_z_global;
end

H_k = zeros(3,7);

if type == 1
    H_k(1,4) = -2*x_kk1(6) * acc_z_global;
    H_k(1,5) = 2*x_kk1(7) * acc_z_global;
    H_k(1,6) = -2*x_kk1(4) * acc_z_global;
    H_k(1,7) = 2*x_kk1(5) * acc_z_global;
    
    H_k(2,4) = 2*x_kk1(5) * acc_z_global;
    H_k(2,5) = 2*x_kk1(4) * acc_z_global;
    H_k(2,6) = 2*x_kk1(7) * acc_z_global;
    H_k(2,7) = 2*x_kk1(6) * acc_z_global;
    
    H_k(3,4) = 0;
    H_k(3,5) = -4*x_kk1(5) * acc_z_global;
    H_k(3,6) = -4*x_kk1(6) * acc_z_global;
    H_k(3,7) = 0;
else
    H_k(1,4) = -2*x_kk1(6) * acc_z_global;
    H_k(1,5) = 2*x_kk1(7) * acc_z_global;
    H_k(1,6) = -2*x_kk1(4) * acc_z_global;
    H_k(1,7) = 2*x_kk1(5) * acc_z_global;
    
    H_k(2,4) = 2*x_kk1(5) * acc_z_global;
    H_k(2,5) = 2*x_kk1(4) * acc_z_global;
    H_k(2,6) = 2*x_kk1(7) * acc_z_global;
    H_k(2,7) = 2*x_kk1(6) * acc_z_global;
    
    H_k(3,4) = 2*x_kk1(4) * acc_z_global;
    H_k(3,5) = -2*x_kk1(5) * acc_z_global;
    H_k(3,6) = -2*x_kk1(6) * acc_z_global;
    H_k(3,7) = 2*x_kk1(7) * acc_z_global;
end
y_k = z_acc - h_kk1;

S_k = (H_k * P_kk1 * H_k') + R_acc;

K_acc = P_kk1*H_k'/(S_k);

x_kk = x_kk1 + K_acc*y_k;

normX = norm(x_kk(4:7),2);
x_kk(4:7) = x_kk(4:7) / normX;

P_kk = (eye(7) - K_acc*H_k)*P_kk1;
