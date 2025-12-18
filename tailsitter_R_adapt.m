function R_new = tailsitter_R_adapt(R)
% R: 3x3 DCM
% 按C++规则输出R_new

R_new = R;
% move z to x  (xcol = zcol)
R_new(:,1) = R(:,3);
% move x to z  (zcol = xcol)
R_new(:,3) = R(:,1);
% change direction of pitch (negate new xcol)
R_new(:,1) = -R_new(:,1);
end