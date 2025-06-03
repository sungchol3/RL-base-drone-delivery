clear; clc; close all;

%% --- 1. 시뮬레이션 파라미터 정의 ---
% 드론 제원
drone_spec.drone_mass = 30.0; % kg
drone_spec.payload_mass = 1.0; % kg
% 관성 모멘트 (실제 드론 값으로 수정 필요)
drone_spec.inertia = diag([0.5, 0.5, 1.0]); % 예시: Ix, Iy, Iz (kg*m^2)

% 미션 웨이포인트 정의 [X, Y, Z_altitude] (시각화 좌표계 기준)
mission_waypoints = [-10, 0, 10;
                      0,  0,  0];
                  
% 초기 위치 및 자세 [X, Y, Z_altitude, Roll, Pitch, Yaw] (m, radians)
% 여기서 X,Y,Z_altitude는 uavScenario의 X,Y,Up 축과 유사하게 사용
initial_pose_xyz_rpy = [mission_waypoints(1,1), mission_waypoints(1,2), mission_waypoints(1,3), 0, 0, 0]; 

% 비행 파라미터
flight_params.update_rate = 100; % Hz, 시뮬레이션 및 제어 주기

% 시각화 사용 여부
enable_visualization = true;

% etc
gravity = 9.8;

% 비디오 녹화 옵션
video_options.enable = false; % true로 설정하면 녹화 시작
video_options.filename = 'drone_flight_test.mp4';
video_options.framerate = 10; % DroneSimulator 내부의 시각화 업데이트 빈도와 맞추는 것이 좋음

%% --- 1.2. PID 파라미 정의 ---
% --- PID 파라미터 설정 (예시 값, 반드시 튜닝 필요!) ---
pid_params.altitude.Kp = 150;  % 고도 P 게인
pid_params.altitude.Ki = 30;   % 고도 I 게인
pid_params.altitude.Kd = 80;   % 고도 D 게인 (수직 속도에 대한 댐핑)
pid_params.altitude.I_limit = 50; % 적분항 제한

pid_params.pos_N.Kp = 2.0;     % North 위치 P 게인 (출력: 목표 가속도 또는 각도 비율)
pid_params.pos_N.Ki = 0.1;
pid_params.pos_N.Kd = 3.0;     % North 속도에 대한 댐핑
pid_params.pos_N.I_limit = 2;

pid_params.pos_E.Kp = 2.0;     % East 위치 P 게인
pid_params.pos_E.Ki = 0.1;
pid_params.pos_E.Kd = 3.0;     % East 속도에 대한 댐핑
pid_params.pos_E.I_limit = 2;

pid_params.angle_limit = deg2rad(20); % 최대 목표 롤/피치 각도 (20도)

pid_params.roll.Kp  = 8;      % 롤 각도 P 게인
pid_params.roll.Ki  = 0.5;
pid_params.roll.Kd  = 0.8;     % 롤 각속도(p)에 대한 댐핑
pid_params.roll.I_limit = 1;

pid_params.pitch.Kp = 8;     % 피치 각도 P 게인
pid_params.pitch.Ki = 0.5;
pid_params.pitch.Kd = 0.8;     % 피치 각속도(q)에 대한 댐핑
pid_params.pitch.I_limit = 1;

pid_params.yaw.Kp   = 5;       % 요 각도 P 게인
pid_params.yaw.Ki   = 0.2;
pid_params.yaw.Kd   = 0.5;      % 요 각속도(r)에 대한 댐핑
pid_params.yaw.I_limit = 0.5;

% 제어 입력 제한
pid_params.thrust_limit_max = 2 * drone_spec.drone_mass * gravity; % 최대 추력 (예: 중력의 2배)
pid_params.torque_limit_xy = 20;  % 롤/피치 토크 최대값 (Nm) - 튜닝 필요
pid_params.torque_limit_z  = 5;   % 요 토크 최대값 (Nm) - 튜닝 필요

% PID 상태 변수 초기화
pid_state = []; % 첫 호출 시 함수 내에서 초기화됨

%% --- 2. DroneSimulator 객체 생성 ---
drone_sim = DroneSimulator(drone_spec, mission_waypoints, initial_pose_xyz_rpy, flight_params, enable_visualization, video_options);

%% --- 3. 시뮬레이션 루프 ---
total_simulation_time = 20; % 총 시뮬레이션 시간 (초)
num_steps = ceil(total_simulation_time / drone_sim.TimeStep);
%{
disp('--- 메인 시뮬레이션 루프 시작 ---');
for step_idx = 1:num_steps
    % 현재 시간
    current_t = drone_sim.CurrentTime;
    
    % ** 여기에 PID 또는 RL 제어기 로직이 들어갑니다 **
    % 제어기는 drone_sim.CurrentState 또는 drone_sim.getObservation()을 입력으로 받고,
    % control_inputs.F_thrust 와 control_inputs.M_body 를 계산합니다.
    
    % 예시: 호버링을 위한 간단한 제어 (목표 고도: 10m)
    target_altitude = 10;
    current_altitude = -drone_sim.CurrentState.pos_inertial(3); % NED에서 D는 depth, -D가 고도
    altitude_error = target_altitude - current_altitude;
    
    % 간단한 P 제어기 (추력)
    kp_alt = 50; % 게인 값 (튜닝 필요)
    base_thrust = drone_sim.DroneParams.mass * drone_sim.DroneParams.g;
    control_inputs.F_thrust = base_thrust + kp_alt * altitude_error;
    
    % 간단한 자세 유지 P 제어기 (토크) - 목표 자세: 0,0,0
    kp_att = 10; % 게인 값 (튜닝 필요)
    current_eul = drone_sim.CurrentState.eul_angles; % [R; P; Y]
    current_ang_vel = drone_sim.CurrentState.ang_vel_body; % [p; q; r]
    
    % 목표 오일러 각 (예: [0;0;0] - 수평 유지)
    target_eul = [0;0;0]; 
    eul_error = target_eul - current_eul;
    
    % 목표 각속도 (예: [0;0;0] - 정지 상태)
    target_ang_vel = [0;0;0];
    ang_vel_error = target_ang_vel - current_ang_vel;

    % 매우 단순화된 PD 제어 유사 로직 (실제로는 더 정교한 PID 필요)
    control_inputs.M_body = kp_att * eul_error(1:2) - 1.0 * current_ang_vel(1:2); % 롤, 피치 토크
    control_inputs.M_body(3) = kp_att * eul_error(3) - 1.0 * current_ang_vel(3); % 요 토크
    
    % 드론 시뮬레이션 한 스텝 진행
    [new_state, ~] = drone_sim.step(control_inputs);
    
    % (선택) 종료 조건 확인 (예: 목표 지점 도달, 시간 초과 등)
    if current_t >= total_simulation_time
        disp('총 시뮬레이션 시간 도달.');
        break;
    end
    
    % (선택) 로그 출력
    if mod(step_idx, drone_sim.TimeStep * 1000) == 0 % 약 1초마다
        fprintf('시간: %.2f s, 고도: %.2f m, 롤: %.2f rad\n', current_t, -new_state.pos_inertial(3), new_state.eul_angles(1));
    end
end
disp('--- 메인 시뮬레이션 루프 완료 ---');
%}

%% --- PID control system ---
% --- 웨이포인트 관리 ---
% mission_waypoints는 [X,Y,Z_altitude] 형식, 시각화 좌표계 기준
% target_waypoint_NED는 [N,E,D] 형식으로 변환 필요
% X -> N, Y -> E, Z_altitude -> -D
current_waypoint_idx = 1; % 첫 번째 웨이포인트부터 시작 (미션 정의상 두번째 점이 목표)
if size(drone_sim.MissionWaypoints,1) < 2
    error('미션 웨이포인트가 최소 2개 이상이어야 합니다.');
end
% 첫번째 목표 웨이포인트 (미션의 두번째 점)
target_vis_coord = drone_sim.MissionWaypoints(current_waypoint_idx+1, :);
target_waypoint_NED = [target_vis_coord(1); target_vis_coord(2); -target_vis_coord(3)]; % N, E, D

% PID 상태 변수 초기화 (루프 전에 한 번만)
pid_state = []; 

disp('--- PID 제어 시뮬레이션 루프 시작 ---');
for step_idx = 1:num_steps
    current_t = drone_sim.CurrentTime;
    
    % 현재 드론 상태 가져오기
    current_drone_state_for_pid = drone_sim.CurrentState;
    
    % PID 제어 입력 계산
    [control_inputs, pid_state] = calculate_pid_control(current_drone_state_for_pid, ...
                                                        target_waypoint_NED, ...
                                                        drone_sim.DroneParams, ...
                                                        pid_params, ...
                                                        pid_state, ...
                                                        drone_sim.TimeStep);
    
    % 드론 시뮬레이션 한 스텝 진행
    [new_state, ~] = drone_sim.step(control_inputs);
    
    % 웨이포인트 도달 확인 및 다음 웨이포인트 설정
    % (간단한 거리 기반 확인)
    current_pos_NED = new_state.pos_inertial;
    dist_to_target = norm(current_pos_NED - target_waypoint_NED);
    arrival_threshold = 1.0; % 도달 임계값 (m) - 튜닝 필요

    if dist_to_target < arrival_threshold
        fprintf('시간: %.2f s - 웨이포인트 %d ([%.1f, %.1f, 고도 %.1f]) 도달!\n', ...
            current_t, current_waypoint_idx + 1, target_vis_coord(1), target_vis_coord(2), -target_waypoint_NED(3));
        current_waypoint_idx = current_waypoint_idx + 1;
        
        if current_waypoint_idx + 1 > size(drone_sim.MissionWaypoints, 1)
            disp('모든 미션 웨이포인트 도달 완료.');
            break; % 시뮬레이션 종료
        else
            % 다음 목표 웨이포인트 설정
            target_vis_coord = drone_sim.MissionWaypoints(current_waypoint_idx+1, :);
            target_waypoint_NED = [target_vis_coord(1); target_vis_coord(2); -target_vis_coord(3)];
            % PID 상태의 일부(예: 위치 관련 적분항)를 리셋할 수도 있음 (선택적)
            pid_state.pos_N.integral = 0; pid_state.pos_N.prev_error = 0;
            pid_state.pos_E.integral = 0; pid_state.pos_E.prev_error = 0;
        end
    end
    
    % 종료 조건 (시간 초과)
    if current_t >= total_simulation_time
        disp('총 시뮬레이션 시간 도달.');
        break;
    end
    
    % 로그 출력 (필요시)
    if mod(step_idx, round(1/drone_sim.TimeStep)) == 0 % 약 1초마다
         fprintf('T: %.2fs, Alt: %.2fm (Tgt: %.1fm), N: %.1fm (Tgt: %.1fm), E: %.1fm (Tgt: %.1fm), R:%.2f,P:%.2f,Y:%.2f\n', ...
             current_t, -new_state.pos_inertial(3), -target_waypoint_NED(3), ...
             new_state.pos_inertial(1), target_waypoint_NED(1), ...
             new_state.pos_inertial(2), target_waypoint_NED(2),...
             rad2deg(new_state.eul_angles(1)),rad2deg(new_state.eul_angles(2)),rad2deg(new_state.eul_angles(3)));
    end
end
drone_sim.closeVideo(); % !중요!
disp('--- PID 제어 시뮬레이션 루프 완료 ---');

%% --- 4. 결과 확인 ---
simulation_results = drone_sim.getResults();
fprintf('총 비행 시간 (계산됨): %.2f 초\n', simulation_results.TotalTime);

if ~isempty(simulation_results.Position)
    final_pos_ned = simulation_results.Position(end,:); % N, E, D
    fprintf('최종 착륙 위치 (NED): N=%.2f m, E=%.2f m, D=%.2f m (고도=%.2f m)\n', ...
        final_pos_ned(1), final_pos_ned(2), final_pos_ned(3), -final_pos_ned(3));

    % 시간에 따른 고도 변화 플롯
    figure;
    plot(simulation_results.Time, -simulation_results.Position(:,3)); % -D가 고도
    xlabel('시간 (s)');
    ylabel('고도 (Up) (m)');
    title('시간에 따른 드론 고도 변화');
    grid on;

    % 시간에 따른 오일러 각 변화 플롯
    figure;
    plot(simulation_results.Time, rad2deg(simulation_results.EulerAngles));
    legend('Roll (deg)', 'Pitch (deg)', 'Yaw (deg)');
    xlabel('시간 (s)');
    ylabel('각도 (deg)');
    title('시간에 따른 드론 오일러 각 변화');
    grid on;
else
    disp('시뮬레이션 데이터가 없습니다.');
end

% 시각화 창을 유지하고 싶지 않다면 주석 해제
% if drone_sim.EnableVisualization && isgraphics(drone_sim.FigureHandle)
%     close(drone_sim.FigureHandle);
% end%% Our main code.