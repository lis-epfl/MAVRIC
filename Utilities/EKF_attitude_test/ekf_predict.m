function [x_kk1,P_kk1] = ekf_predict(x_k1k1,P_k1k1,dt,w,sigma_r_sqr,sigma_w_sqr)

x_kk1 = zeros(7,1);

x_kk1(1) = x_k1k1(1);
x_kk1(2) = x_k1k1(2);
x_kk1(3) = x_k1k1(3);

x_kk1(4) = x_k1k1(4) + 0.5*dt*(  -(w(1)-x_k1k1(1))*x_k1k1(5) ...
                                -(w(2)-x_k1k1(2))*x_k1k1(6)...
                                -(w(3)-x_k1k1(3))*x_k1k1(7));
x_kk1(5) = x_k1k1(5) + 0.5*dt*(  (w(1)-x_k1k1(1))*x_k1k1(4) ...
                                +(w(3)-x_k1k1(3))*x_k1k1(6) ...
                                -(w(2)-x_k1k1(2))*x_k1k1(7));
x_kk1(6) = x_k1k1(6) + 0.5*dt*(  (w(2)-x_k1k1(2))*x_k1k1(4)...
                                -(w(3)-x_k1k1(3))*x_k1k1(5)...
                                +(w(1)-x_k1k1(1))*x_k1k1(7));
x_kk1(7) = x_k1k1(7) + 0.5*dt*(  (w(3)-x_k1k1(3))*x_k1k1(4)...
                                +(w(2)-x_k1k1(2))*x_k1k1(5)...
                                -(w(1)-x_k1k1(1))*x_k1k1(6));

F = zeros(7,7);

F(1,1) = 1;
F(2,2) = 1;
F(3,3) = 1;

F(4,1) = x_k1k1(5)*dt;
F(4,2) = x_k1k1(6)*dt;
F(4,3) = x_k1k1(7)*dt;
F(4,4) = 1;
F(4,5) = -(w(1)-x_k1k1(1))*dt;
F(4,6) = -(w(2)-x_k1k1(2))*dt;
F(4,7) = -(w(3)-x_k1k1(3))*dt;

F(5,1) = -x_k1k1(4)*dt;
F(5,2) = x_k1k1(7)*dt;
F(5,3) = -x_k1k1(6)*dt;
F(5,4) = (w(1)-x_k1k1(1))*dt;
F(5,5) = 1;
F(5,6) = (w(3)-x_k1k1(3))*dt;
F(5,7) = -(w(2)-x_k1k1(2))*dt;

F(6,1) = -x_k1k1(7)*dt;
F(6,2) = -x_k1k1(4)*dt;
F(6,3) = x_k1k1(5)*dt;
F(6,4) = (w(2)-x_k1k1(2))*dt;
F(6,5) = -(w(3)-x_k1k1(3))*dt;
F(6,6) = 1;
F(6,7) = (w(1)-x_k1k1(1))*dt;

F(7,1) = x_k1k1(6)*dt;
F(7,2) = -x_k1k1(5)*dt;
F(7,3) = -x_k1k1(4)*dt;
F(7,4) = (w(3)-x_k1k1(3))*dt;
F(7,5) = (w(2)-x_k1k1(2))*dt;
F(7,6) = -(w(1)-x_k1k1(1))*dt;
F(7,7) = 1;

normX = norm(x_kk1(4:7),2);
x_kk1(4:7) = x_kk1(4:7) / normX;

Q = zeros(7,7);

dt2_2 = dt^2/2;
dt3_3 = dt^3/3;

Q(1,1) = sigma_w_sqr * dt;
Q(1,2) = 0;
Q(1,3) = 0;
Q(1,4) = x_kk1(5) * dt2_2 * sigma_w_sqr;
Q(1,5) = -x_kk1(4) * dt2_2 * sigma_w_sqr;
Q(1,6) = -x_kk1(7) * dt2_2 * sigma_w_sqr;
Q(1,7) = x_kk1(6) * dt2_2 * sigma_w_sqr;

Q(2,1) = 0;
Q(2,2) = sigma_w_sqr * dt;
Q(2,3) = 0;
Q(2,4) = x_kk1(6) * dt2_2 * sigma_w_sqr;
Q(2,5) = x_kk1(7) * dt2_2 * sigma_w_sqr;
Q(2,6) = -x_kk1(4) * dt2_2 * sigma_w_sqr;
Q(2,7) = -x_kk1(5) * dt2_2 * sigma_w_sqr;

Q(3,1) = 0;
Q(3,2) = 0;
Q(3,3) = sigma_w_sqr * dt;
Q(3,4) = x_kk1(7) * dt2_2 * sigma_w_sqr;
Q(3,5) = -x_kk1(6) * dt2_2 * sigma_w_sqr;
Q(3,6) = x_kk1(5) * dt2_2 * sigma_w_sqr;
Q(3,7) = -x_kk1(4) * dt2_2 * sigma_w_sqr;

Q(4,1) = x_kk1(5) * dt2_2 * sigma_w_sqr;
Q(4,2) = x_kk1(6) * dt2_2 * sigma_w_sqr;
Q(4,3) = x_kk1(7) * dt2_2 * sigma_w_sqr;
Q(4,4) = sigma_r_sqr*dt + sigma_r_sqr*dt3_3*norm(w)^2 + (x_kk1(5)^2 + x_kk1(6)^2 + x_kk1(7)^2)*dt3_3*sigma_w_sqr;
Q(4,5) = -x_kk1(4)*x_kk1(5)*dt3_3*sigma_w_sqr;
Q(4,6) = -x_kk1(4)*x_kk1(6)*dt3_3*sigma_w_sqr;
Q(4,7) = -x_kk1(4)*x_kk1(7)*dt3_3*sigma_w_sqr;

Q(5,1) = -x_kk1(4) * dt2_2 * sigma_w_sqr;
Q(5,2) = x_kk1(7) * dt2_2 * sigma_w_sqr;
Q(5,3) = -x_kk1(6) * dt2_2 * sigma_w_sqr;
Q(5,4) = -x_kk1(5)*x_kk1(4)*dt3_3*sigma_w_sqr;
Q(5,5) = sigma_r_sqr*dt + sigma_r_sqr*dt3_3*norm(w)^2 + (x_kk1(4)^2 + x_kk1(6)^2 + x_kk1(7)^2)*dt3_3*sigma_w_sqr;
Q(5,6) = -x_kk1(5)*x_kk1(6)*dt3_3*sigma_w_sqr;
Q(5,7) = -x_kk1(5)*x_kk1(7)*dt3_3*sigma_w_sqr;

Q(6,1) = -x_kk1(7) * dt2_2 * sigma_w_sqr;
Q(6,2) = -x_kk1(4) * dt2_2 * sigma_w_sqr;
Q(6,3) = x_kk1(5) * dt2_2 * sigma_w_sqr;
Q(6,4) = -x_kk1(6)*x_kk1(4)*dt3_3*sigma_w_sqr;
Q(6,5) = -x_kk1(6)*x_kk1(5)*dt3_3*sigma_w_sqr;
Q(6,6) = sigma_r_sqr*dt + sigma_r_sqr*dt3_3*norm(w)^2 + (x_kk1(4)^2 + x_kk1(5)^2 + x_kk1(7)^2)*dt3_3*sigma_w_sqr;
Q(6,7) = -x_kk1(6)*x_kk1(7)*dt3_3*sigma_w_sqr;

Q(7,1) = x_kk1(6) * dt2_2 * sigma_w_sqr;
Q(7,2) = -x_kk1(5) * dt2_2 * sigma_w_sqr;
Q(7,3) = -x_kk1(4) * dt2_2 * sigma_w_sqr;
Q(7,4) = -x_kk1(7)*x_kk1(4)*dt3_3*sigma_w_sqr;
Q(7,5) = -x_kk1(7)*x_kk1(5)*dt3_3*sigma_w_sqr;
Q(7,6) = -x_kk1(7)*x_kk1(6)*dt3_3*sigma_w_sqr;
Q(7,7) = sigma_r_sqr*dt + sigma_r_sqr*dt3_3*norm(w)^2 + (x_kk1(4)^2 + x_kk1(5)^2 + x_kk1(6)^2)*dt3_3*sigma_w_sqr;

P_kk1 = F*P_k1k1*F' + Q;