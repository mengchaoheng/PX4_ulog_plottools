%% PX4算法移植 vs MATLAB R2025a 一致性验证 
% 重点验证: 
%   1. q.euler() 与 q.rotmat() 对象方法
%   2. 内置函数 quat2eul/quat2rotm 与自定义函数的对应关系
%   3. 物理转换 V_n -> V_b 的正确性 (rotateframe vs R^T * v)

clear; clc; close all;

%% ================= [0] 数据准备 =================
N = 10; 
disp(['>>> [0] 生成 ', num2str(N), ' 组测试数据...']);

% 1. 模拟 PX4 四元数 (Body -> NED) [w, x, y, z]
q_rand = rand(N, 4);
q_obj_b2n = normalize(quaternion(q_rand)); 
q_arr_b2n = compact(q_obj_b2n); % Nx4 数组

% 2. 模拟 NED 系速度 V_n
v_n = rand(N, 3) * 20;

% 3. 计算"真值" (使用你的 C++ 移植逻辑)
% [C++逻辑 1] R_b2n
R_custom = quat2dcm_batch(q_arr_b2n); 
% [C++逻辑 2] Euler [Roll, Pitch, Yaw]
eul_custom = dcm2euler_batch(R_custom);
% [C++逻辑 3] V_b = R^T * V_n (矩阵转置法)
v_b_custom = zeros(N, 3);
for i=1:N, v_b_custom(i,:) = (R_custom(:,:,i)' * v_n(i,:)')'; end

disp('    数据准备完成。真值已建立。');
disp('------------------------------------------------------------');

%% ================= [1] 物理验证: 速度向量变换 V_n -> V_b =================
disp('>>> [1] 物理验证: V_b 计算准确性 (目标: 必须等于 C++ 真值)');

% 1.1 使用 rotateframe (神器)
% 物理含义: 坐标系旋转投影。q_b2n 代表坐标系从 B 转到 N。
% rotateframe(q, v) 计算 v 在原坐标系(B)下的投影。
v_b_rotateframe = rotateframe(q_obj_b2n, v_n);

% 1.2 使用 rotatepoint (需手动求逆)
% 物理含义: 向量旋转。
% 要从 N 转回 B，需要用反向旋转 q_n2b = conj(q_b2n)
v_b_rotatepoint = rotatepoint(conj(q_obj_b2n), v_n);

% --- 验证结果 ---
err_rf = norm(v_b_custom - v_b_rotateframe, 'fro');
err_pt = norm(v_b_custom - v_b_rotatepoint, 'fro');

fprintf('    rotateframe(q, v_n):        %s (Err: %.2e) [最推荐]\n', check(err_rf), err_rf);
fprintf('    rotatepoint(conj(q), v_n):  %s (Err: %.2e)\n', check(err_pt), err_pt);
disp(' ');

%% ================= [2] 对象方法验证: rotmat =================
disp('>>> [2] 对象验证: q.rotmat() vs quat2rotm vs 自定义DCM');

% 2.1 内置函数 quat2rotm
R_func = quat2rotm(q_arr_b2n);

% 2.2 对象方法 rotmat (point模式)
% 'point' 模式通常对应标准的旋转矩阵 (v_new = R * v_old)
R_obj_point = rotmat(q_obj_b2n, 'point');

% 2.3 对象方法 rotmat (frame模式)
R_obj_frame = rotmat(q_obj_b2n, 'frame');

% --- 验证结果 ---
% 验证 A: 自定义 DCM vs rotmat('point')
err_custom_point = max(abs(R_custom(:) - R_obj_point(:)));
% 验证 B: quat2rotm vs rotmat('point')
err_func_point   = max(abs(R_func(:)   - R_obj_point(:)));
% 验证 C: rotmat('frame') 是什么? (通常是 point 的转置)
% 检查是否 R_frame == R_custom'
R_custom_T = permute(R_custom, [2, 1, 3]);
err_custom_frame = max(abs(R_custom_T(:) - R_obj_frame(:)));

fprintf('    自定义 R_b2n == q.rotmat("point"): %s (Err: %.2e)\n', check(err_custom_point), err_custom_point);
fprintf('    quat2rotm    == q.rotmat("point"): %s (Err: %.2e)\n', check(err_func_point), err_func_point);
if err_custom_frame < 1e-10
    fprintf('    [注] q.rotmat("frame") 等于 R_b2n 的转置 (R_n2b)。\n');
end
disp(' ');

%% ================= [3] 对象方法验证: euler =================
disp('>>> [3] 对象验证: q.euler() vs quat2eul vs 自定义欧拉角');
disp('    [重要提示] PX4/C++输出顺序: [Roll, Pitch, Yaw]');
disp('    [重要提示] MATLAB 输出顺序: [Yaw, Pitch, Roll] (ZYX顺序)');

% 3.1 内置函数 quat2eul (默认 ZYX)
eul_func = quat2eul(q_arr_b2n); 

% 3.2 对象方法 euler (指定 ZYX, frame模式)
eul_obj = euler(q_obj_b2n, 'ZYX', 'frame');

% --- 验证结果 ---
% 1. 验证函数与对象方法是否一致
err_func_obj = norm(eul_func - eul_obj, 'fro');

% 2. 验证与自定义 C++ 的关系 (需要交换列顺序)
% MATLAB [Y, P, R] -> 变为 [R, P, Y]
eul_obj_reordered = [eul_obj(:,3), eul_obj(:,2), eul_obj(:,1)];
err_custom_obj = norm(eul_custom - eul_obj_reordered, 'fro');

fprintf('    quat2eul == q.euler("ZYX"):      %s (Err: %.2e)\n', check(err_func_obj), err_func_obj);
fprintf('    自定义[RPY] == 翻转(q.euler):    %s (Err: %.2e)\n', check(err_custom_obj), err_custom_obj);
disp(' ');

%% ================= [4] 最终结论 =================
disp('============================================================');
disp('                    验 证 总 结                              ');
disp('============================================================');
if err_rf < 1e-10 && err_custom_point < 1e-10 && err_custom_obj < 1e-5
    disp('PASS: 所有验证通过！');
    disp('1. 转换矩阵: 你的 quat2dcm 对应 q.rotmat(..., "point")');
    disp('2. 欧拉角:   你的 [Roll,Pitch,Yaw] 对应 q.euler(...,"ZYX") 的列翻转 [3,2,1]');
    disp('3. 速度变换: 使用 v_b = rotateframe(q_b2n, v_n) 是正确且最高效的。');
else
    disp('FAIL: 存在不一致，请检查上方报错信息。');
end


%% --- 辅助函数 ---
function s = check(err)
    if err < 1e-5, s = 'PASS'; else, s = 'FAIL'; end
end

% --- C++ 移植函数 (保持不变) ---
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