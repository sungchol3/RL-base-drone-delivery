clear; clc; close all;

%% --- 1. 시뮬레이션 파라미터 정의 ---
% 드론 제원
drone_spec.drone_mass = 30.0; % kg
drone_spec.payload_mass = 1.0; % kg
% 관성 모멘트 (실제 드론 값으로 수정 필요)
drone_spec.inertia = diag([0.5, 0.5, 1.0]); % 예시: Ix, Iy, Iz (kg*m^2)

% 미션 웨이포인트 정의 [X, Y, Z_altitude] (시각화 좌표계 기준)
mission_waypoints = [-10, 0, 10;
                      0,  0, 10;
                      0,  0,  0];
                  
% 초기 위치 및 자세 [X, Y, Z_altitude, Roll, Pitch, Yaw] (m, radians)
% 여기서 X,Y,Z_altitude는 uavScenario의 X,Y,Up 축과 유사하게 사용
initial_pose_xyz_rpy = [mission_waypoints(1,1), mission_waypoints(1,2), mission_waypoints(1,3), 0, 0, 0]; 

% 비행 파라미터
flight_params.update_rate = 100; % Hz, 시뮬레이션 및 제어 주기

% 시각화 사용 여부
enable_visualization = true;

%% --- 2. DroneSimulator 객체 생성 ---
drone_sim = DroneSimulator(drone_spec, mission_waypoints, initial_pose_xyz_rpy, flight_params, enable_visualization);

%% --- 3. 시뮬레이션 루프 ---
total_simulation_time = 20; % 총 시뮬레이션 시간 (초)
num_steps = ceil(total_simulation_time / drone_sim.TimeStep);

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