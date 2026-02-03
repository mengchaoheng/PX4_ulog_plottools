%% PX4 Algorithm Porting vs MATLAB R2025a Consistency Verification 
% Key verification: 
%   1. q.euler() and q.rotmat() object methods
%   2. Correspondence between built-in functions quat2eul/quat2rotm and custom functions
%   3. Correctness of physical transformation V_n -> V_b (rotateframe vs R^T * v)

clear; clc; close all;

%% ================= [0] Data Preparation =================
N = 10; 
disp(['>>> [0] Generate ', num2str(N), ' sets of test data...']);

% 1. Simulate PX4 quaternion (Body -> NED) [w, x, y, z]
q_rand = rand(N, 4);
q_obj_b2n = normalize(quaternion(q_rand)); 
q_arr_b2n = compact(q_obj_b2n); % Nx4 array

% 2. Simulate NED frame velocity V_n
v_n = rand(N, 3) * 20;

% 3. Calculate "ground truth" (using your C++ porting logic)
% [C++ logic 1] R_b2n
R_custom = quat2dcm_batch(q_arr_b2n); 
% [C++ logic 2] Euler [Roll, Pitch, Yaw]
eul_custom = dcm2euler_batch(R_custom);
% [C++ logic 3] V_b = R^T * V_n (matrix transpose method)
v_b_custom = zeros(N, 3);
for i=1:N, v_b_custom(i,:) = (R_custom(:,:,i)' * v_n(i,:)')'; end

disp('    Data preparation complete. Ground truth established.');
disp('------------------------------------------------------------');

%% ================= [1] Physical Verification: Velocity Vector Transform V_n -> V_b =================
disp('>>> [1] Physical verification: V_b calculation accuracy (target: must equal C++ ground truth)');

% 1.1 Use rotateframe (magic tool)
% Physical meaning: coordinate system rotation projection. q_b2n represents coordinate system transformation from B to N.
% rotateframe(q, v) calculates projection of v in original coordinate system (B).
v_b_rotateframe = rotateframe(q_obj_b2n, v_n);

% 1.2 Use rotatepoint (need manual inverse)
% Physical meaning: vector rotation.
% To transform from N back to B, need to use reverse rotation q_n2b = conj(q_b2n)
v_b_rotatepoint = rotatepoint(conj(q_obj_b2n), v_n);

% --- Verification results ---
err_rf = norm(v_b_custom - v_b_rotateframe, 'fro');
err_pt = norm(v_b_custom - v_b_rotatepoint, 'fro');

fprintf('    rotateframe(q, v_n):        %s (Err: %.2e) [Most recommended]\n', check(err_rf), err_rf);
fprintf('    rotatepoint(conj(q), v_n):  %s (Err: %.2e)\n', check(err_pt), err_pt);
disp(' ');

%% ================= [2] Object Method Verification: rotmat =================
disp('>>> [2] Object verification: q.rotmat() vs quat2rotm vs custom DCM');

% 2.1 Built-in function quat2rotm
R_func = quat2rotm(q_arr_b2n);

% 2.2 Object method rotmat (point mode)
% 'point' mode usually corresponds to standard rotation matrix (v_new = R * v_old)
R_obj_point = rotmat(q_obj_b2n, 'point');

% 2.3 Object method rotmat (frame mode)
R_obj_frame = rotmat(q_obj_b2n, 'frame');

% --- Verification results ---
% Verification A: Custom DCM vs rotmat('point')
err_custom_point = max(abs(R_custom(:) - R_obj_point(:)));
% Verification B: quat2rotm vs rotmat('point')
err_func_point   = max(abs(R_func(:)   - R_obj_point(:)));
% Verification C: What is rotmat('frame')? (usually transpose of point)
% Check if R_frame == R_custom'
R_custom_T = permute(R_custom, [2, 1, 3]);
err_custom_frame = max(abs(R_custom_T(:) - R_obj_frame(:)));

fprintf('    Custom R_b2n == q.rotmat("point"): %s (Err: %.2e)\n', check(err_custom_point), err_custom_point);
fprintf('    quat2rotm    == q.rotmat("point"): %s (Err: %.2e)\n', check(err_func_point), err_func_point);
if err_custom_frame < 1e-10
    fprintf('    [Note] q.rotmat("frame") equals transpose of R_b2n (R_n2b).\n');
end
disp(' ');

%% ================= [3] Object Method Verification: euler =================
disp('>>> [3] Object verification: q.euler() vs quat2eul vs custom Euler angles');
disp('    [Important Note] PX4/C++ output order: [Roll, Pitch, Yaw]');
disp('    [Important Note] MATLAB output order: [Yaw, Pitch, Roll] (ZYX order)');

% 3.1 Built-in function quat2eul (default ZYX)
eul_func = quat2eul(q_arr_b2n); 

% 3.2 Object method euler (specify ZYX, frame mode)
eul_obj = euler(q_obj_b2n, 'ZYX', 'frame');

% --- Verification results ---
% 1. Verify if function and object method are consistent
err_func_obj = norm(eul_func - eul_obj, 'fro');

% 2. Verify relationship with custom C++ (need to swap column order)
% MATLAB [Y, P, R] -> change to [R, P, Y]
eul_obj_reordered = [eul_obj(:,3), eul_obj(:,2), eul_obj(:,1)];
err_custom_obj = norm(eul_custom - eul_obj_reordered, 'fro');

fprintf('    quat2eul == q.euler("ZYX"):      %s (Err: %.2e)\n', check(err_func_obj), err_func_obj);
fprintf('    Custom[RPY] == flipped(q.euler):    %s (Err: %.2e)\n', check(err_custom_obj), err_custom_obj);
disp(' ');

%% ================= [4] Final Conclusion =================
disp('============================================================');
disp('                    Verification Summary                              ');
disp('============================================================');
if err_rf < 1e-10 && err_custom_point < 1e-10 && err_custom_obj < 1e-5
    disp('PASS: All verifications passed!');
    disp('1. Rotation matrix: Your quat2dcm corresponds to q.rotmat(..., "point")');
    disp('2. Euler angles:   Your [Roll,Pitch,Yaw] corresponds to q.euler(...,"ZYX") with column flip [3,2,1]');
    disp('3. Velocity transform: Using v_b = rotateframe(q_b2n, v_n) is correct and most efficient.');
else
    disp('FAIL: Inconsistencies exist, please check error information above.');
end


%% --- Helper Functions ---
function s = check(err)
    if err < 1e-5, s = 'PASS'; else, s = 'FAIL'; end
end

% --- C++ Porting Functions (keep unchanged) ---
function dcm = quat2dcm_batch(q)
    N = size(q, 1);
    w=q(:,1); x=q(:,2); y=q(:,3); z=q(:,4);
    dcm = zeros(3, 3, N);
    dcm(1,1,:) = 1 - 2*(y.^2 + z.^2); dcm(1,2,:) = 2*(x.*y - w.*z);   dcm(1,3,:) = 2*(x.*z + w.*y);
    dcm(2,1,:) = 2*(x.*y + w.*z);   dcm(2,2,:) = 1 - 2*(x.^2 + z.^2); dcm(2,3,:) = 2*(y.*z - w.*x);
    dcm(3,1,:) = 2*(x.*z - w.*y);   dcm(3,2,:) = 2*(y.*z + w.*x);   dcm(3,3,:) = 1 - 2*(x.^2 + y.^2);
end

function euler = dcm2euler_batch(dcm)
    N = size(dcm, 3); euler = zeros(N, 3);
    r31 = squeeze(dcm(3,1,:)); r32 = squeeze(dcm(3,2,:)); r33 = squeeze(dcm(3,3,:));
    r21 = squeeze(dcm(2,1,:)); r11 = squeeze(dcm(1,1,:));
    euler(:, 1) = atan2(r32, r33); % phi
    euler(:, 2) = asin(-r31);      % theta
    euler(:, 3) = atan2(r21, r11); % psi
end