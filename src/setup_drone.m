%% Define Function setup_drone
function results = setup_drone(drone_spec, mission_waypoints, initial_pose_xyz_rpy, flight_params, enable_visualization)
% setup_drone: 드론 시뮬레이션 환경을 설정하고, 주어진 경로를 비행하며 결과를 반환합니다.

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

disp(initial_pose_xyz_rpy(1:3))

% 1단계: UAV 객체만 사용하여 플랫폼 생성
platform = uavPlatform("UAV",scenario, ...
    InitialPosition=[initial_pose_xyz_rpy(2),initial_pose_xyz_rpy(1),-initial_pose_xyz_rpy(3)],...
    InitialOrientation=initial_orientation_quat);

disp('Calling setup(scenario)...');

setup(scenario); % 시나리오 및 포함된 모든 플랫폼/센서 초기화
disp('setup(scenario) complete.');

disp('플랫폼 초기 위치 및 자세 설정 완료.');

% update Platform Mesh
disp('Defining constant arguments for updateMesh...');
type_arg_um = 'quadrotor';
drone_scale_factor_um = 1.5; % 드론 크기
geometries_arg_um = {drone_scale_factor_um};
color_arg_um = [0.2 0.3 0.8]; % 드론 색상

% updateMesh의 position과 orientation은 플랫폼 기준 상대값이므로,
% 드론 몸체 자체를 나타내는 메쉬는 항상 [0,0,0] 위치와 단위 쿼터니언 방향을 가집니다.
relative_position_um = [0 0 0]; 
relative_orientation_um_quat = [1 0 0 0]; % 단위 쿼터니언 [w, x, y, z]


updateMesh(platform, ...
           type_arg_um, ...
           geometries_arg_um, ...
           color_arg_um, ...
           relative_position_um, ...
           relative_orientation_um_quat);
disp('Initial updateMesh call successful.');

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
previous_pos = current_pose_xyz;
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
        % simple level flight (you can change if using PID or RL control
        % system here)
        next_target_for_yaw = mission_waypoints(waypoint_index, :);
        delta_pos_for_yaw = next_target_for_yaw(1:2) - current_pose_xyz(1:2);
        if norm(delta_pos_for_yaw) > 1e-6 
            yaw_angle = atan2(delta_pos_for_yaw(2), delta_pos_for_yaw(1));
            current_orientation_rpy(3) = yaw_angle; 
        end
        current_orientation_rpy(1:2) = 0; 
    end

    % 1. 현재 위치(position) : current_pose_xyz -> [Y, X, -Z]
    update_pos = [current_pose_xyz(2), current_pose_xyz(1), -current_pose_xyz(3)];
    % 2. 현재 속도(velcoity)
    current_vel = (current_pose_xyz - previous_pos) ./ dt;

    % 3. 현재 가속도(acceleration) : 중요하지 않으므로 [0,0,0]으로 설정
    current_acc = [0, 0, 0];

    % 4. Quaternion vector [qw qx qy qz]
    %    current_orientation_rpy는 [Roll, Pitch, Yaw] 순서
    current_eul_zyx = [current_orientation_rpy(3), current_orientation_rpy(2), current_orientation_rpy(1)]; % [Yaw, Pitch, Roll]
    current_eul_quat = eul2quat(current_eul_zyx,"ZYX");

    % 5. Angular velocity [wx wy wz]

    if step_count == 1 || dt == 0
        angvel_3elements = [0, 0, 0];
    else
        previous_eul_zyx = trajectory(5:7); % [R P Y]
        previous_eul_quat = eul2quat(previous_eul_zyx, "ZYX");
        q_dot = (current_eul_quat - previous_eul_quat) / dt;
        omega_b_intermediate = quatmultiply(quatconj(previous_eul_quat), q_dot);
        omega_b_pure_quaternion = 2 * omega_b_intermediate;
        
        angvel_3elements = omega_b_pure_quaternion(2:4);
    end
    
    %%% Move drone object
    motion_input_16elements = [update_pos, current_vel, current_acc, current_eul_quat, angvel_3elements];
    move(platform, motion_input_16elements);

    MotionInfo = read(platform);
    % currentPosition = motionInfo;


    current_time = current_time + dt;
    trajectory(step_count, :) = [current_time, current_pose_xyz, current_orientation_rpy];

    if enable_visualization && mod(step_count, 20) == 0 
        show3D(scenario);
        xlabel(ax_3d, 'X (m)'); ylabel(ax_3d, 'Y (m)'); zlabel(ax_3d, 'Z (m)');
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