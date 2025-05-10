clear all; close all; clc;
%% Test flight and visualization
% --- 1. 시뮬레이션 파라미터 정의 ---
% 드론 제원
drone_spec.drone_mass = 1.0; % kg
drone_spec.payload_mass = 5.0; % kg

% 미션 웨이포인트 정의 [x, y, z]
% (-10,0,10) -> (0,0,10) -> (0,0,0)
mission_waypoints = [-10, 0, 10;
                      0,  0, 10;
                      0,  0,  0];

% 초기 위치 및 자세 [x, y, z, roll, pitch, yaw] (m, radians)
% 첫 번째 웨이포인트에서 시작, 초기 자세는 모두 0 (수평, 북쪽 또는 x축 방향)
initial_pose_xyz_rpy = [mission_waypoints(1,1), mission_waypoints(1,2), mission_waypoints(1,3), 0, 0, 0];

% 비행 파라미터
flight_params.speed = 2.0;       % m/s (기본 비행 로직용)
flight_params.update_rate = 100; % Hz, 시나리오 업데이트 및 데이터 로깅 주기

% 시각화 사용 여부
enable_visualization = true;

% --- 2. setup_drone 함수 호출 ---
simulation_results = setup_drone(drone_spec, mission_waypoints, initial_pose_xyz_rpy, flight_params, enable_visualization);

% --- 3. 결과 확인 (예시) ---
disp('--- Main Script: 시뮬레이션 결과 ---');
fprintf('총 비행 시간: %.2f 초\n', simulation_results.total_time);

if ~isempty(simulation_results.trajectory)
    final_position = simulation_results.trajectory(end, 2:4);
    fprintf('최종 착륙 위치: X=%.2f m, Y=%.2f m, Z=%.2f m\n', final_position(1), final_position(2), final_position(3));

    % 추가적인 결과 그래프 (예: 시간에 따른 고도 변화)
    figure;
    plot(simulation_results.trajectory(:,1), simulation_results.trajectory(:,4));
    xlabel('시간 (s)');
    ylabel('고도 Z (m)');
    title('시간에 따른 드론 고도 변화');
    grid on;
else
    disp('시뮬레이션 데이터가 없습니다.');
end

% 만약 시각화 창을 닫고 싶다면:
% if isfield(simulation_results, 'figure_handle') && ishandle(simulation_results.figure_handle)
%     % close(simulation_results.figure_handle);
% end