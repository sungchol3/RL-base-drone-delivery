%% Define Function setup_drone
function results = setup_drone(drone_spec, mission_waypoints, initial_pose_xyz_rpy, flight_params, enable_visualization)
% setup_drone: 드론 시뮬레이션 환경을 설정하고, 주어진 경로를 비행하며 결과를 반환합니다.
% ... (함수 설명 주석은 이전과 동일) ...

disp('--- 드론 설정 및 기본 비행 시뮬레이션 시작 ---');

% --- 1. 드론 및 시나리오 초기화 ---
total_mass = drone_spec.drone_mass + drone_spec.payload_mass;

% 시나리오 생성
scenario = uavScenario(UpdateRate=flight_params.update_rate, ReferenceLocation = [0 0 0]);

% 드론 객체 생성 (multirotor)
drone = multirotor;

% Configuration 구조체 설정
disp('drone.Configuration 설정 시도...');
drone_config_struct.PDRoll   = [10, 2];   % 예시: [Kp_roll, Kd_roll]
drone_config_struct.PDPitch  = [10, 2];   % 예시: [Kp_pitch, Kd_pitch]
drone_config_struct.PYawRate = 5;       % 예시: Kp_yaw_rate (스칼라)
drone_config_struct.PThrust  = 2;         % 예시: Kp_thrust (스칼라)
drone_config_struct.Mass = total_mass; % 계산된 총 질량

drone.Configuration = drone_config_struct;
disp('drone.Configuration 설정 완료.');

% 플랫폼(드론)을 시나리오에 추가
initial_orientation_eul_zyx = [initial_pose_xyz_rpy(6), initial_pose_xyz_rpy(5), initial_pose_xyz_rpy(4)]; % [Yaw, Pitch, Roll]
initial_orientation_quat = eul2quat(initial_orientation_eul_zyx, 'ZYX');

disp('uavPlatform 생성 및 시나리오 연결 시도...');

% 1단계: UAV 객체만 사용하여 플랫폼 생성
platform = uavPlatform("UAV",scenario, ...
    InitialPosition=initial_pose_xyz_rpy(1:3),...
    InitialOrientation=initial_orientation_quat);

disp('플랫폼 초기 위치 및 자세 설정 완료.');


% 시뮬레이션 시간 스텝
dt = 1 / flight_params.update_rate;
current_time = 0;

% 결과 저장을 위한 배열 초기화
trajectory = [];
max_simulation_steps = 10000; 
trajectory = zeros(max_simulation_steps, 7); % [time, x, y, z, R, P, Y]

% --- 2. 시각화 설정 (활성화된 경우) ---
figure_handle = [];
if enable_visualization
    figure_handle = figure;
    ax_3d = show3D(scenario); 
    hold(ax_3d, 'on');
    plot3(ax_3d, mission_waypoints(:,1), mission_waypoints(:,2), mission_waypoints(:,3), ...
          'ro-', 'MarkerFaceColor','r', 'DisplayName','미션 경로');
    title(ax_3d, '드론 비행 시뮬레이션');
    xlabel(ax_3d, 'X (m)'); ylabel(ax_3d, 'Y (m)'); zlabel(ax_3d, 'Z (m)');
    legend(ax_3d, 'show', 'Location', 'best');
    grid(ax_3d, 'on');
    axis(ax_3d, 'equal'); 
    view(ax_3d, 30, 20); 
    drawnow;
end

% --- 3. 기본 경로 추종 비행 시뮬레이션 ---
current_pose_xyz = initial_pose_xyz_rpy(1:3);
current_orientation_rpy = initial_pose_xyz_rpy(4:6); % [R, P, Y]

num_waypoints = size(mission_waypoints, 1);
waypoint_index = 1;
step_count = 0;

disp('경로 추종 비행 시작...');
while waypoint_index <= num_waypoints && step_count < max_simulation_steps
    step_count = step_count + 1;
    target_waypoint = mission_waypoints(waypoint_index, :);

    vector_to_target = target_waypoint - current_pose_xyz;
    distance_to_target = norm(vector_to_target);

    if distance_to_target < flight_params.speed * dt * 1.5 
        current_pose_xyz = target_waypoint; 
        waypoint_index = waypoint_index + 1;
        if waypoint_index <= num_waypoints 
             fprintf('웨이포인트 %d 도달: [%.2f, %.2f, %.2f]\n', waypoint_index-1, target_waypoint(1), target_waypoint(2), target_waypoint(3));
        else
            fprintf('최종 웨이포인트 도달: [%.2f, %.2f, %.2f]\n', target_waypoint(1), target_waypoint(2), target_waypoint(3));
            disp('모든 웨이포인트 도달 완료.');
            break; 
        end
    else
        move_vector = (vector_to_target / distance_to_target) * flight_params.speed * dt;
        current_pose_xyz = current_pose_xyz + move_vector;
    end

    if waypoint_index <= num_waypoints 
        next_target_for_yaw = mission_waypoints(waypoint_index, :);
        delta_pos_for_yaw = next_target_for_yaw(1:2) - current_pose_xyz(1:2);
        if norm(delta_pos_for_yaw) > 1e-6 
            yaw_angle = atan2(delta_pos_for_yaw(2), delta_pos_for_yaw(1));
            current_orientation_rpy(3) = yaw_angle; 
        end
        current_orientation_rpy(1:2) = 0; 
    end

    % 1. 현재 자세(오일러 각)를 ZYX 순서로 준비: [Yaw, Pitch, Roll]
    current_eul_zyx = [current_orientation_rpy(3), current_orientation_rpy(2), current_orientation_rpy(1)];
    
    % 2. 오일러 각을 회전 행렬로 변환
    rotation_matrix = eul2rotm(current_eul_zyx, 'ZYX'); % ZYX 순서
    
    % 3. 4x4 동차 변환 행렬 생성
    %   T = [ R(3x3)  P(3x1) ]
    %       [ 0(1x3)    1    ]
    transform_matrix_4x4 = eye(4); % 단위 행렬로 시작
    transform_matrix_4x4(1:3, 1:3) = rotation_matrix;
    transform_matrix_4x4(1:3, 4) = current_pose_xyz'; % 위치 벡터 (열벡터로)
    
    % 4. 4x4 행렬을 1x16 벡터로 변환 (열 우선 전치)
    motion_input_16elements = transform_matrix_4x4(:)';
    
    move(platform, motion_input_16elements);

    current_time = current_time + dt;
    trajectory(step_count, :) = [current_time, current_pose_xyz, current_orientation_rpy];

    if enable_visualization && mod(step_count, 5) == 0 
        show3D(scenario); 
        title(ax_3d, sprintf('드론 비행 시뮬레이션 (시간: %.2fs)', current_time));
        drawnow limitrate; 
    end
end

trajectory = trajectory(1:step_count, :);
total_time = current_time;

fprintf('총 비행 시간: %.2f 초\n', total_time);

results.trajectory = trajectory;
results.total_time = total_time;
if enable_visualization && ishandle(figure_handle) 
    results.figure_handle = figure_handle;
    plot3(ax_3d, trajectory(:,2), trajectory(:,3), trajectory(:,4), 'b-', 'LineWidth',1.5, 'DisplayName','실제 비행 경로');
    legend(ax_3d, 'show', 'Location', 'best'); 
    hold(ax_3d, 'off');
end

disp('--- 드론 설정 및 기본 비행 시뮬레이션 완료 ---');
end