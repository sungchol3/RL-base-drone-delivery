function [control_inputs, pid_state_new] = calculate_pid_control(current_state_sim, target_waypoint_NED, drone_params, pid_params, prev_pid_state, dt)
% calculate_pid_control: PID 제어 로직을 통해 드론의 추력과 토크를 계산합니다.
%
% 입력:
%   current_state_sim: DroneSimulator의 현재 상태 구조체
%       .pos_inertial: [N; E; D] (m)
%       .vel_inertial: [Vn; Ve; Vd] (m/s)
%       .eul_angles:   [Roll; Pitch; Yaw] (rad)
%       .ang_vel_body: [p; q; r] (rad/s) - 동체 기준 각속도
%   target_waypoint_NED: [N_target; E_target; D_target] 목표 웨이포인트 (m) - NED 좌표계
%   drone_params:      드론 물리 파라미터 (.mass, .g)
%   pid_params:        PID 게인 값들을 담은 구조체
%                      .altitude, .pos_N, .pos_E, .roll, .pitch, .yaw 각각에 .Kp, .Ki, .Kd 필드
%   prev_pid_state:    이전 스텝의 PID 상태 (적분항, 이전 오차)
%   dt:                시뮬레이션 시간 스텝 (s)
%
% 출력:
%   control_inputs:    구조체 .F_thrust (N), .M_body [Mx;My;Mz] (Nm)
%   pid_state_new:     업데이트된 PID 상태

    % 현재 상태 추출
    pos_N = current_state_sim.pos_inertial(1);
    pos_E = current_state_sim.pos_inertial(2);
    pos_D = current_state_sim.pos_inertial(3); % Down is positive
    
    vel_N = current_state_sim.vel_inertial(1);
    vel_E = current_state_sim.vel_inertial(2);
    vel_D = current_state_sim.vel_inertial(3);
    
    roll_current  = current_state_sim.eul_angles(1);
    pitch_current = current_state_sim.eul_angles(2);
    yaw_current   = current_state_sim.eul_angles(3);
    
    p_current = current_state_sim.ang_vel_body(1); % body roll rate
    q_current = current_state_sim.ang_vel_body(2); % body pitch rate
    r_current = current_state_sim.ang_vel_body(3); % body yaw rate

    % 목표 웨이포인트 추출
    N_target = target_waypoint_NED(1);
    E_target = target_waypoint_NED(2);
    D_target = target_waypoint_NED(3); % Down is positive, so -Altitude

    % PID 상태 초기화 (만약 비어있다면)
    if isempty(prev_pid_state)
        fields = {'altitude', 'pos_N', 'pos_E', 'roll', 'pitch', 'yaw'};
        for i = 1:length(fields)
            prev_pid_state.(fields{i}).integral = 0;
            prev_pid_state.(fields{i}).prev_error = 0;
        end
    end
    pid_state_new = prev_pid_state;

    % --- 1. 고도 제어 (Altitude Control) -> F_thrust ---
    altitude_current = -pos_D; % Altitude is positive up
    altitude_target  = -D_target;
    altitude_error   = altitude_target - altitude_current;
    
    pid_state_new.altitude.integral = prev_pid_state.altitude.integral + altitude_error * dt;
    pid_state_new.altitude.integral = max(min(pid_state_new.altitude.integral, pid_params.altitude.I_limit), -pid_params.altitude.I_limit); % Anti-windup
    altitude_derivative = (altitude_error - prev_pid_state.altitude.prev_error) / dt;
    pid_state_new.altitude.prev_error = altitude_error;
    
    % 고도 PID 출력 (수직 속도 -Vd를 D항으로 사용 가능)
    thrust_cmd_from_pid = pid_params.altitude.Kp * altitude_error + ...
                          pid_params.altitude.Ki * pid_state_new.altitude.integral + ...
                          pid_params.altitude.Kd * (-vel_D); % Derivative on measurement (-vel_D is upward velocity)
                          % pid_params.altitude.Kd * altitude_derivative; 
    
    % 중력 보상 및 피치/롤 각에 따른 추력 보정 (수직 방향 추력 성분)
    % small angle approximation: cos(roll) ~ 1, cos(pitch) ~ 1 for hover
    % More accurate: base_thrust / (cos(roll_current)*cos(pitch_current))
    hover_thrust = drone_params.mass * drone_params.g;
    F_thrust = hover_thrust / (cos(roll_current) * cos(pitch_current)) + thrust_cmd_from_pid;
    F_thrust = max(0, min(F_thrust, pid_params.thrust_limit_max)); % 추력 제한

    % --- 2. 수평 위치 제어 (Position Control) -> 목표 롤/피치 각도 ---
    % 목표: target_N, target_E. 현재: pos_N, pos_E
    % 이 PID의 출력은 목표 가속도가 되고, 이를 목표 롤/피치 각도로 변환
    
    % North (X) 방향 제어 -> 목표 피치각
    pos_N_error = N_target - pos_N;
    pid_state_new.pos_N.integral = prev_pid_state.pos_N.integral + pos_N_error * dt;
    pid_state_new.pos_N.integral = max(min(pid_state_new.pos_N.integral, pid_params.pos_N.I_limit), -pid_params.pos_N.I_limit);
    pos_N_derivative = (pos_N_error - prev_pid_state.pos_N.prev_error) / dt;
    pid_state_new.pos_N.prev_error = pos_N_error;
    
    % 이 PID의 출력을 '목표 가속도'로 해석하여 각도로 변환. (또는 직접 목표 각도로)
    % 가속도 명령 (m/s^2)
    acc_N_desired = pid_params.pos_N.Kp * pos_N_error + ...
                    pid_params.pos_N.Ki * pid_state_new.pos_N.integral + ...
                    pid_params.pos_N.Kd * vel_N; % Derivative on measurement (or pos_N_derivative)
                    % pid_params.pos_N.Kd * pos_N_derivative;

    % East (Y) 방향 제어 -> 목표 롤각
    pos_E_error = E_target - pos_E;
    pid_state_new.pos_E.integral = prev_pid_state.pos_E.integral + pos_E_error * dt;
    pid_state_new.pos_E.integral = max(min(pid_state_new.pos_E.integral, pid_params.pos_E.I_limit), -pid_params.pos_E.I_limit);
    pos_E_derivative = (pos_E_error - prev_pid_state.pos_E.prev_error) / dt;
    pid_state_new.pos_E.prev_error = pos_E_error;
    
    acc_E_desired = pid_params.pos_E.Kp * pos_E_error + ...
                    pid_params.pos_E.Ki * pid_state_new.pos_E.integral + ...
                    pid_params.pos_E.Kd * vel_E; % Derivative on measurement (or pos_E_derivative)
                    % pid_params.pos_E.Kd * pos_E_derivative;

    % 목표 가속도를 목표 롤/피치 각도로 변환 (NED, FRD 기준)
    % F_thrust가 대략 m*g 라고 가정, 작은 각도 가정
    % acc_N_desired (앞으로 가속) -> 피치 음수 (코가 아래로)
    % acc_E_desired (오른쪽으로 가속) -> 롤 양수 (오른쪽으로 기울임)
    % psi = yaw_current
    % 매핑: (더 정확한 방법은 힘 벡터를 회전시키는 것)
    % pitch_desired = (1/g) * (acc_N_desired * cos(psi) + acc_E_desired * sin(psi));
    % roll_desired  = (1/g) * (acc_N_desired * sin(psi) - acc_E_desired * cos(psi)); 
    % 위 공식은 힘벡터 F_xy를 현재 yaw에 대해 정렬했을 때.
    % 더 간단하게, 현재 yaw를 무시하고 월드 프레임 가속도 명령으로 각도 생성
    % (월드 N 가속은 바디 x 가속, 월드 E 가속은 바디 y 가속으로 가정, yaw=0일때)
    pitch_desired = - acc_N_desired / drone_params.g; % North 가속 위해 pitch down(-) (FRD body: -X_body)
    roll_desired  =   acc_E_desired / drone_params.g; % East 가속 위해 roll right(+) (FRD body: +Y_body)
    
    % 목표 각도 제한
    pitch_desired = max(min(pitch_desired, pid_params.angle_limit), -pid_params.angle_limit);
    roll_desired  = max(min(roll_desired, pid_params.angle_limit), -pid_params.angle_limit);

    % --- 3. 자세 제어 (Attitude Control) -> Mx, My, Mz 토크 ---
    % 롤 제어
    roll_error = roll_desired - roll_current;
    pid_state_new.roll.integral = prev_pid_state.roll.integral + roll_error * dt;
    pid_state_new.roll.integral = max(min(pid_state_new.roll.integral, pid_params.roll.I_limit), -pid_params.roll.I_limit);
    % roll_derivative = (roll_error - prev_pid_state.roll.prev_error) / dt; % 이것보다 p_current 사용이 일반적
    pid_state_new.roll.prev_error = roll_error;
    Mx_torque = pid_params.roll.Kp * roll_error + ...
                pid_params.roll.Ki * pid_state_new.roll.integral - ...
                pid_params.roll.Kd * p_current; % D 항은 각속도(p_current)에 대한 댐핑

    % 피치 제어
    pitch_error = pitch_desired - pitch_current;
    pid_state_new.pitch.integral = prev_pid_state.pitch.integral + pitch_error * dt;
    pid_state_new.pitch.integral = max(min(pid_state_new.pitch.integral, pid_params.pitch.I_limit), -pid_params.pitch.I_limit);
    % pitch_derivative = (pitch_error - prev_pid_state.pitch.prev_error) / dt;
    pid_state_new.pitch.prev_error = pitch_error;
    My_torque = pid_params.pitch.Kp * pitch_error + ...
                pid_params.pitch.Ki * pid_state_new.pitch.integral - ...
                pid_params.pitch.Kd * q_current; % D 항은 각속도(q_current)에 대한 댐핑

    % 요 제어 (다음 웨이포인트를 향하도록)
    yaw_desired = atan2(E_target - pos_E, N_target - pos_N); % NED 기준
    yaw_error = yaw_desired - yaw_current;
    % 각도 오차 정규화 (-pi ~ pi)
    yaw_error = atan2(sin(yaw_error), cos(yaw_error));
    
    pid_state_new.yaw.integral = prev_pid_state.yaw.integral + yaw_error * dt;
    pid_state_new.yaw.integral = max(min(pid_state_new.yaw.integral, pid_params.yaw.I_limit), -pid_params.yaw.I_limit);
    % yaw_derivative = (yaw_error - prev_pid_state.yaw.prev_error) / dt;
    pid_state_new.yaw.prev_error = yaw_error;
    Mz_torque = pid_params.yaw.Kp * yaw_error + ...
                pid_params.yaw.Ki * pid_state_new.yaw.integral - ...
                pid_params.yaw.Kd * r_current; % D 항은 각속도(r_current)에 대한 댐핑

    % 토크 제한
    Mx_torque = max(min(Mx_torque, pid_params.torque_limit_xy), -pid_params.torque_limit_xy);
    My_torque = max(min(My_torque, pid_params.torque_limit_xy), -pid_params.torque_limit_xy);
    Mz_torque = max(min(Mz_torque, pid_params.torque_limit_z), -pid_params.torque_limit_z);

    % 제어 입력 구조체 생성
    control_inputs.F_thrust = F_thrust;
    control_inputs.M_body   = [Mx_torque; My_torque; Mz_torque];
end