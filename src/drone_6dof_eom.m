function [pos_new, vel_new, acc_linear_inertial, eul_new, ang_vel_body_new] = drone_6dof_eom(current_state, control_inputs, drone_params, dt)
% drone_6dof_eom: 6-DOF 드론 운동 방정식을 사용하여 다음 시간 스텝의 상태를 계산합니다.
%
% 입력 (Inputs):
%   current_state: 현재 드론 상태를 담고 있는 구조체(struct)
%       .pos_inertial: [3x1] 현재 위치 벡터 (x, y, z) - 관성 좌표계 기준 (m)
%       .vel_inertial: [3x1] 현재 선형 속도 벡터 (vx, vy, vz) - 관성 좌표계 기준 (m/s)
%       .eul_angles:   [3x1] 현재 오일러 각 [롤; 피치; 요] (phi, theta, psi) - (rad)
%       .ang_vel_body: [3x1] 현재 각속도 벡터 (p, q, r) - 동체 좌표계 기준 (rad/s)
%
%   control_inputs: 제어 입력을 담고 있는 구조체(struct)
%       .F_thrust:  (scalar) 총 추력 크기 (N), 드론 동체 Z축의 음수 방향으로 작용 (위로 밀어 올림)
%       .M_body:    [3x1] 동체 기준 토크 벡터 [Mx; My; Mz] (롤, 피치, 요 토크) (N*m)
%
%   drone_params: 드론 파라미터를 담고 있는 구조체(struct)
%       .mass: (scalar) 총 질량 (kg)
%       .I:    [3x3] 관성 모멘트 텐서 (kg*m^2) - 동체 좌표계 기준, 주축에 대해 대각행렬 가정 가능
%       .g:    (scalar) 중력 가속도 (m/s^2, 예: 9.81)
%
%   dt: (scalar) 시뮬레이션 시간 스텝 (s)
%
% 출력 (Outputs):
%   pos_new:             [3x1] 다음 스텝 위치 벡터 (x, y, z) - 관성 좌표계 기준 (m)
%   vel_new:             [3x1] 다음 스텝 선형 속도 벡터 (vx, vy, vz) - 관성 좌표계 기준 (m/s)
%   acc_linear_inertial: [3x1] 현재 스텝 선형 가속도 벡터 (ax, ay, az) - 관성 좌표계 기준 (m/s^2)
%   eul_new:             [3x1] 다음 스텝 오일러 각 [롤; 피치; 요] (rad)
%   ang_vel_body_new:    [3x1] 다음 스텝 각속도 벡터 (p, q, r) - 동체 좌표계 기준 (rad/s)
%
% 좌표계 가정:
%   관성 좌표계 (Inertial Frame): NED (North-East-Down) - Z축이 아래를 향함
%   동체 좌표계 (Body Frame): FRD (Front-Right-Down) - X축이 앞, Y축이 오른쪽, Z축이 아래를 향함
%                            추력은 동체 -Z축 방향(위)으로 작용한다고 가정

% 현재 상태 추출
pos_inertial = current_state.pos_inertial; % [x; y; z]
vel_inertial = current_state.vel_inertial; % [vx; vy; vz]
eul_angles   = current_state.eul_angles;   % [phi; theta; psi] (roll, pitch, yaw)
ang_vel_body = current_state.ang_vel_body; % [p; q; r]

% 제어 입력 추출
F_thrust_magnitude = control_inputs.F_thrust;
M_body             = control_inputs.M_body;     % [Mx; My; Mz]

% 드론 파라미터 추출
m = drone_params.mass;
I = drone_params.I; % 3x3 관성 행렬
g = drone_params.g;

% --- 1. 회전 행렬 계산 (동체 좌표계 -> 관성 좌표계) ---
phi   = eul_angles(1); % roll
theta = eul_angles(2); % pitch
psi   = eul_angles(3); % yaw

R_x_phi = [1, 0,        0;
           0, cos(phi), -sin(phi);
           0, sin(phi), cos(phi)];

R_y_theta = [cos(theta), 0, sin(theta);
             0,          1, 0;
             -sin(theta),0, cos(theta)];

R_z_psi = [cos(psi), -sin(psi), 0;
           sin(psi), cos(psi),  0;
           0,        0,         1];

% ZYX 순서 (Yaw, Pitch, Roll 순으로 회전 적용)
R_body_to_inertial = R_z_psi * R_y_theta * R_x_phi;

% --- 2. 힘 계산 및 선형 운동 방정식 ---
% 중력 (관성 좌표계 NED에서 Z축 양의 방향)
F_gravity_inertial = [0; 0; m*g];

% 추력 (동체 좌표계 FRD에서 -Z축 방향, 즉 위로 작용)
F_thrust_body = [0; 0; -F_thrust_magnitude];
F_thrust_inertial = R_body_to_inertial * F_thrust_body;

% 총 힘 (관성 좌표계)
F_total_inertial = F_gravity_inertial + F_thrust_inertial;
% (공기저항 등 추가적인 힘이 있다면 여기에 더함)

% 선형 가속도 (관성 좌표계)
acc_linear_inertial = F_total_inertial / m;

% 선형 속도 및 위치 업데이트 (오일러 적분)
vel_new = vel_inertial + acc_linear_inertial * dt;
pos_new = pos_inertial + vel_inertial * dt + 0.5 * acc_linear_inertial * dt^2; % 좀 더 정확한 위치 업데이트


% --- 3. 토크 및 회전 운동 방정식 (오일러 방정식) ---
% ang_vel_body = [p; q; r]
% M_body = [Mx; My; Mz]
% I * dw/dt + w x (Iw) = M_body
% dw/dt = inv(I) * (M_body - w x (Iw))

omega_cross_I_omega = cross(ang_vel_body, I * ang_vel_body);
ang_acc_body = I \ (M_body - omega_cross_I_omega); % I \ A 는 inv(I)*A 와 동일

% 각속도 업데이트 (오일러 적분)
ang_vel_body_new = ang_vel_body + ang_acc_body * dt;


% --- 4. 오일러 각 업데이트 (운동학적 관계식) ---
% eul_dot = W * omega_body
% W = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
%      0, cos(phi),            -sin(phi);
%      0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

% 각도의 특이점(Gimbal lock)을 피하기 위해 cos(theta)가 0에 가까워지는 것을 주의해야 함
% 실제로는 Quaternion 기반으로 자세를 업데이트하는 것이 더 강건함
% 여기서는 요청에 따라 오일러 각을 사용

W_eul_rate_transform = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
                        0, cos(phi),            -sin(phi);
                        0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

eul_rates = W_eul_rate_transform * ang_vel_body; % [phi_dot; theta_dot; psi_dot]

% 오일러 각 업데이트 (오일러 적분)
eul_new = eul_angles + eul_rates * dt;

% 요 각(psi)을 -pi ~ pi 범위로 정규화 (선택 사항)
% eul_new(3) = atan2(sin(eul_new(3)), cos(eul_new(3)));

end