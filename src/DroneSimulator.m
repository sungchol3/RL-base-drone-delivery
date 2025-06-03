classdef DroneSimulator < handle % handle 클래스를 상속받으면 객체 참조가 용이해짐
    % DroneSimulator: 드론의 물리적 시뮬레이션 및 시나리오 관리를 위한 클래스
    
    properties
        % 시나리오 및 시각화 관련
        Scenario              % UAV Scenario 객체
        DronePlatform         % UAV Platform 객체 (드론의 시각적 표현 및 시나리오 내 위치)
        EnableVisualization   % 시각화 사용 여부 (true/false)
        FigureHandle          % 시각화 창 핸들
        AxesHandle            % 3D 플롯 축 핸들
        WaypointPlotHandle    % 웨이포인트 플롯 핸들
        TrajectoryPlotHandle  % 실제 비행 경로 플롯 핸들
        
        % 드론 물리적 파라미터
        DroneParams           % 구조체: .mass, .I (관성행렬), .g (중력)
        
        % 드론 상태 변수
        CurrentState          % 구조체: .pos_inertial, .vel_inertial, .eul_angles, .ang_vel_body
        
        % 미션 및 시뮬레이션 파라미터
        InitialPoseXYZRPY     % [x,y,z,roll,pitch,yaw] 초기 위치 및 자세
        MissionWaypoints      % [Nx3] 웨이포인트 매트릭스 [x,y,z]
        TimeStep              % (dt) 시뮬레이션 시간 간격 (s)
        CurrentTime           % 현재 시뮬레이션 시간 (s)
        
        % 로깅
        TrajectoryLog         % [Mx7] 시간, 위치(3), 오일러각(3) 저장 배열
        MaxLogSteps           % 로그 배열의 최대 크기
        CurrentLogIndex       % 현재 로그 인덱스
    end
    
    methods
        % --- 생성자 ---
        function obj = DroneSimulator(drone_spec, mission_waypoints, initial_pose_xyz_rpy, flight_params, enable_visualization)
            % 입력:
            %   drone_spec: .drone_mass, .payload_mass, .inertia (선택적, 없으면 기본값)
            %   mission_waypoints: 웨이포인트
            %   initial_pose_xyz_rpy: 초기 위치/자세
            %   flight_params: .update_rate (Hz)
            %   enable_visualization: 시각화 여부

            disp('--- DroneSimulator 객체 초기화 시작 ---');
            
            % 드론 물리 파라미터 설정
            obj.DroneParams.mass = drone_spec.drone_mass + drone_spec.payload_mass;
            if isfield(drone_spec, 'inertia') && isequal(size(drone_spec.inertia), [3,3])
                obj.DroneParams.I = drone_spec.inertia;
            else
                % 예시 기본 관성 모멘트 (실제 값으로 교체 필요)
                disp('경고: 드론 관성 모멘트가 제공되지 않아 기본값을 사용합니다.');
                obj.DroneParams.I = diag([0.2, 0.2, 0.4]); % Ix, Iy, Iz (kg*m^2)
            end
            obj.DroneParams.g = 9.81;
            
            % 미션 및 시뮬레이션 파라미터 저장
            obj.InitialPoseXYZRPY = initial_pose_xyz_rpy;
            obj.MissionWaypoints = mission_waypoints;
            obj.TimeStep = 1 / flight_params.update_rate;
            obj.EnableVisualization = enable_visualization;
            
            % 시나리오 생성 (UpdateRate는 여기서 직접 제어하므로 scenario에는 기본값 사용 가능)
            obj.Scenario = uavScenario('ReferenceLocation', [0 0 0]); % UpdateRate는 step 메서드에서 관리

            % 초기 상태 설정 (NED 좌표계 기준)
            % initial_pose_xyz_rpy는 [X,Y,Z_altitude, R,P,Y]로 가정
            % CurrentState.pos_inertial은 [N; E; D] (North, East, Down)
            obj.CurrentState.pos_inertial = [initial_pose_xyz_rpy(1); initial_pose_xyz_rpy(2); -initial_pose_xyz_rpy(3)]; % Z_altitude -> -D
            obj.CurrentState.vel_inertial = [0; 0; 0];
            obj.CurrentState.eul_angles   = [initial_pose_xyz_rpy(4); initial_pose_xyz_rpy(5); initial_pose_xyz_rpy(6)]; % [Roll; Pitch; Yaw]
            obj.CurrentState.ang_vel_body = [0; 0; 0];
            
            % 플랫폼(드론)을 시나리오에 추가
            % uavPlatform의 InitialPosition은 [East, North, Up] 또는 [X,Y,Z_altitude] 형식일 수 있음.
            % 원본 setup_drone.m의 InitialPosition=[initial_pose_xyz_rpy(2),initial_pose_xyz_rpy(1),-initial_pose_xyz_rpy(3)]을 따름
            % 이는 Y, X, Z_Down (NED의 N, E, D를 Y,X,Z_platform으로 매핑)
            % 즉, initial_pose_xyz_rpy(1)=N, (2)=E, (3)=Altitude 이므로,
            % Platform InitialPosition은 [E, N, Altitude] 가 됨.
            platform_initial_pos = [initial_pose_xyz_rpy(2), initial_pose_xyz_rpy(1), initial_pose_xyz_rpy(3)];
            
            initial_orientation_eul_zyx = [initial_pose_xyz_rpy(6), initial_pose_xyz_rpy(5), initial_pose_xyz_rpy(4)]; % [Yaw, Pitch, Roll]
            initial_orientation_quat = eul2quat(initial_orientation_eul_zyx, 'ZYX');

            obj.DronePlatform = uavPlatform("UAV", obj.Scenario, ...
                'InitialPosition', platform_initial_pos, ...
                'InitialOrientation', initial_orientation_quat);
            
            % 드론 메쉬 업데이트 (원본 setup_drone.m 참조)
            updateMesh(obj.DronePlatform, 'quadrotor', {1.5}, [0.2 0.3 0.8], [0 0 0], [1 0 0 0]);

            % 시나리오 초기화 (플랫폼 추가 후)
            setup(obj.Scenario);

            % 로깅 및 시간 초기화
            obj.MaxLogSteps = ceil(200 / obj.TimeStep); % 예: 최대 200초 분량, 필요시 조절
            obj.reset(); % 시간, 로그, 상태 초기화

            % 시각화 설정
            if obj.EnableVisualization
                obj.setupVisualization();
            end
            disp('--- DroneSimulator 객체 초기화 완료 ---');
        end
        
        % --- 시뮬레이션 스텝 진행 ---
        function [currentState, linear_accel_inertial] = step(obj, control_inputs)
            % 입력:
            %   control_inputs: 구조체 .F_thrust (N), .M_body [Mx;My;Mz] (Nm)
            % 출력:
            %   currentState: 업데이트된 현재 상태 구조체
            %   linear_accel_inertial: 계산된 관성계 선형 가속도
            
            % 6-DOF 운동 방정식 호출 (이 클래스의 static 메서드 또는 별도 .m 파일)
            [pos_new, vel_new, acc_lin, eul_new, ang_vel_new] = ...
                DroneSimulator.drone_6dof_eom_static(obj.CurrentState, control_inputs, obj.DroneParams, obj.TimeStep);
            
            % 현재 상태 업데이트
            obj.CurrentState.pos_inertial = pos_new;
            obj.CurrentState.vel_inertial = vel_new;
            obj.CurrentState.eul_angles   = eul_new;
            obj.CurrentState.ang_vel_body = ang_vel_new;
            
            % UAV 플랫폼 시각적 위치/자세 업데이트
            % drone_6dof_eom_static의 pos_new는 [N; E; D]
            % eul_new는 [Roll; Pitch; Yaw]
            % move_platform 메서드 내부에서 좌표 변환 및 16요소 벡터 생성
            obj.move_platform_visuals(pos_new, vel_new, acc_lin, eul_new, ang_vel_new);
            
            % 시간 업데이트
            obj.CurrentTime = obj.CurrentTime + obj.TimeStep;
            
            % 로깅
            if obj.CurrentLogIndex <= obj.MaxLogSteps
                obj.TrajectoryLog(obj.CurrentLogIndex, :) = [obj.CurrentTime, pos_new', eul_new'];
                obj.CurrentLogIndex = obj.CurrentLogIndex + 1;
            end
            
            % 시각화 업데이트 (매 스텝 또는 주기적으로)
            if obj.EnableVisualization && mod(obj.CurrentLogIndex, 10) == 0 % 예: 10 스텝마다 업데이트
                 obj.updateVisualization();
            end
            
            currentState = obj.CurrentState; % 업데이트된 상태 반환
            linear_accel_inertial = acc_lin;
        end

        % --- 시뮬레이션 리셋 ---
        function reset(obj)
            obj.CurrentTime = 0;
            % 초기 상태로 리셋 (NED 좌표계)
            obj.CurrentState.pos_inertial = [obj.InitialPoseXYZRPY(1); obj.InitialPoseXYZRPY(2); -obj.InitialPoseXYZRPY(3)];
            obj.CurrentState.vel_inertial = [0; 0; 0];
            obj.CurrentState.eul_angles   = [obj.InitialPoseXYZRPY(4); obj.InitialPoseXYZRPY(5); obj.InitialPoseXYZRPY(6)];
            obj.CurrentState.ang_vel_body = [0; 0; 0];
            
            % 플랫폼 위치 리셋 (uavPlatform은 [E,N,Up] 또는 [X,Y,Z_alt] 사용)
            platform_initial_pos = [obj.InitialPoseXYZRPY(2), obj.InitialPoseXYZRPY(1), obj.InitialPoseXYZRPY(3)];
            initial_orientation_eul_zyx = [obj.InitialPoseXYZRPY(6), obj.InitialPoseXYZRPY(5), obj.InitialPoseXYZRPY(4)];
            initial_orientation_quat = eul2quat(initial_orientation_eul_zyx, 'ZYX');
            
            % 플랫폼 리셋을 위해서는 Scenario를 다시 setup 하거나,
            % 플랫폼의 Trajectory 속성을 직접 수정하는 방법이 필요할 수 있음.
            % 가장 간단한 방법은 객체를 새로 생성하는 것이지만, reset 기능을 원한다면
            % uavPlatform의 상태를 초기화하는 방법을 찾아야 함.
            % 여기서는 간단히 CurrentState만 리셋하고, move_platform_visuals가 초기 상태를 반영하도록 함.
            obj.move_platform_visuals(obj.CurrentState.pos_inertial, ...
                                      obj.CurrentState.vel_inertial, ...
                                      [0;0;0], ... % 초기 가속도는 0으로 가정
                                      obj.CurrentState.eul_angles, ...
                                      obj.CurrentState.ang_vel_body);

            obj.TrajectoryLog = zeros(obj.MaxLogSteps, 7); % [t, N, E, D, R, P, Y]
            obj.CurrentLogIndex = 1;

            disp(isgraphics(obj.AxesHandle))
            
            if obj.EnableVisualization && all(isgraphics(obj.AxesHandle))
                % 기존 궤적 지우기 (새로운 핸들로 다시 그리거나, 기존 핸들의 데이터 교체)
                if isgraphics(obj.TrajectoryPlotHandle)
                    delete(obj.TrajectoryPlotHandle);
                    obj.TrajectoryPlotHandle = []; % 핸들 초기화
                end
                title(obj.AxesHandle, '드론 비행 시뮬레이션 (초기화됨)');
                drawnow;
            end
            disp('DroneSimulator가 초기 상태로 리셋되었습니다.');
        end
        
        % --- 관측값 반환 (RL 에이전트용) ---
        function obs = getObservation(obj)
            % RL 에이전트가 사용할 관측값을 현재 상태로부터 구성하여 반환
            % 예시: [pos_inertial; vel_inertial; eul_angles; ang_vel_body]
            % 필요에 따라 정규화 또는 특정 요소만 선택 가능
            obs = [obj.CurrentState.pos_inertial; 
                   obj.CurrentState.vel_inertial; 
                   obj.CurrentState.eul_angles; 
                   obj.CurrentState.ang_vel_body];
        end

        % --- 시뮬레이션 결과 반환 ---
        function results = getResults(obj)
            results.Time = obj.TrajectoryLog(1:obj.CurrentLogIndex-1, 1);
            results.Position = obj.TrajectoryLog(1:obj.CurrentLogIndex-1, 2:4); % N, E, D
            results.EulerAngles = obj.TrajectoryLog(1:obj.CurrentLogIndex-1, 5:7); % R, P, Y
            results.TotalTime = obj.CurrentTime;
        end

    end
    
    methods (Access = private)
        % --- 시각화 설정 ---
        function setupVisualization(obj)
            obj.FigureHandle = figure;
            obj.AxesHandle = show3D(obj.Scenario);
            hold(obj.AxesHandle, 'on');
            % 웨이포인트 플롯 (선택적: mission_waypoints가 있다면)
            if ~isempty(obj.MissionWaypoints)
                 % mission_waypoints는 [X,Y,Z_altitude] 형식으로 가정
                 % show3D의 좌표계는 uavScenario의 ReferenceLocation을 따르므로,
                 % 일반적으로 East-North-Up (ENU) 또는 North-East-Up (NEU)을 따름.
                 % mission_waypoints가 NED 기준이라면 변환 필요.
                 % 여기서는 mission_waypoints가 X,Y,Z (아마도 ENU 또는 NEU)라고 가정.
                 % 원본 setup_drone.m의 plot3(ax_3d, mission_waypoints(:,1), mission_waypoints(:,2), mission_waypoints(:,3), ...)를 따름
                obj.WaypointPlotHandle = plot3(obj.AxesHandle, obj.MissionWaypoints(:,1), obj.MissionWaypoints(:,2), obj.MissionWaypoints(:,3), ...
                                               'ro-', 'MarkerFaceColor','r', 'DisplayName','미션 경로');
            end
            title(obj.AxesHandle, '드론 비행 시뮬레이션');
            xlabel(obj.AxesHandle, 'X (East) (m)'); % uavScenario 기본 축 이름 따름
            ylabel(obj.AxesHandle, 'Y (North) (m)');
            zlabel(obj.AxesHandle, 'Z (Up) (m)');
            legend(obj.AxesHandle, 'show', 'Location', 'best');
            grid(obj.AxesHandle, 'on');
            axis(obj.AxesHandle, 'equal'); 
            view(obj.AxesHandle, 30, 20); 
            drawnow;
        end
        
        % --- 시각화 업데이트 ---
        function updateVisualization(obj)
            if obj.EnableVisualization && isgraphics(obj.AxesHandle)
                show3D(obj.Scenario); % 이것만으로도 플랫폼 위치가 업데이트 될 수 있음
                % 하지만, 좀 더 동적인 타이틀 업데이트 등을 위해 drawnow 사용
                title(obj.AxesHandle, sprintf('드론 비행 시뮬레이션 (시간: %.2fs)', obj.CurrentTime));
                drawnow limitrate; % limitrate는 과도한 업데이트 방지
            end
        end

        % --- 플랫폼 시각적 이동 ---
        function move_platform_visuals(obj, pos_ned, vel_ned, acc_ned, eul_rpy, ang_vel_rpy_body)
            % 입력:
            %   pos_ned: [N; E; D]
            %   vel_ned: [Vn; Ve; Vd]
            %   acc_ned: [An; Ae; Ad]
            %   eul_rpy: [Roll; Pitch; Yaw]
            %   ang_vel_rpy_body: [p; q; r] (body frame roll, pitch, yaw rates)

            % uavPlatform.move()는 16요소 벡터를 요구함
            % [posX, posY, posZ, velX, velY, velZ, accX, accY, accZ, qW, qX, qY, qZ, angVelX, angVelY, angVelZ]
            % 좌표계 변환: NED -> 플랫폼의 [Y,X,-Z] (East, North, Up)와 유사한 형태
            
            % 위치: NED [N,E,D] -> 플랫폼 입력 [E, N, -D(Up)]
            platform_pos = [pos_ned(2); pos_ned(1); -pos_ned(3)];
            
            % 속도: NED [Vn,Ve,Vd] -> 플랫폼 입력 [Ve, Vn, -Vd(Up)]
            platform_vel = [vel_ned(2); vel_ned(1); -vel_ned(3)];

            % 가속도: NED [An,Ae,Ad] -> 플랫폼 입력 [Ae, An, -Ad(Up)]
            platform_acc = [acc_ned(2); acc_ned(1); -acc_ned(3)];
            
            % 자세: 오일러각 [Roll,Pitch,Yaw] -> 쿼터니언 [w,x,y,z]
            % eul2quat은 ZYX 순서의 오일러각([yaw, pitch, roll])을 입력으로 받음
            eul_zyx_for_quat = [eul_rpy(3); eul_rpy(2); eul_rpy(1)]; % Yaw, Pitch, Roll
            quat_wxyz = eul2quat(eul_zyx_for_quat', 'ZYX'); % eul2quat은 행벡터 입력을 선호할 수 있음

            % 각속도: 동체 기준 [p,q,r] (이미 동체 기준이므로 플랫폼 입력에 적합할 수 있음, 순서 주의)
            % move 명령의 angVel은 월드인지 바디인지, 순서가 RPY인지 XYZ인지 확인 필요.
            % 원본 setup_drone.m에서는 angvel_3elements를 [wx wy wz]로 사용.
            % 이것이 동체 각속도 p,q,r과 어떤 관계인지 명확히 해야 함.
            % 여기서는 [p;q;r]을 그대로 사용한다고 가정. (순서가 XYZ일 수 있음)
            platform_ang_vel_body = ang_vel_rpy_body; % [p;q;r]

            motion_input_16elements = [platform_pos', ...
                                       platform_vel', ...
                                       platform_acc', ...
                                       quat_wxyz, ... % quat_wxyz는 이미 [w,x,y,z] 순서의 행벡터
                                       platform_ang_vel_body'];
            
            move(obj.DronePlatform, motion_input_16elements);
        end
    end

    methods (Static)
        % --- 6DOF 운동 방정식 (이전에 제공된 함수를 static 메서드로 포함) ---
        function [pos_new, vel_new, acc_linear_inertial, eul_new, ang_vel_body_new] = ...
                drone_6dof_eom_static(current_state, control_inputs, drone_params, dt)
            
            pos_inertial = current_state.pos_inertial;
            vel_inertial = current_state.vel_inertial;
            eul_angles   = current_state.eul_angles;
            ang_vel_body = current_state.ang_vel_body;
            
            F_thrust_magnitude = control_inputs.F_thrust;
            M_body             = control_inputs.M_body;
            
            m = drone_params.mass;
            I = drone_params.I;
            g = drone_params.g;
            
            phi   = eul_angles(1); theta = eul_angles(2); psi   = eul_angles(3);
            
            R_x_phi = [1,0,0; 0,cos(phi),-sin(phi); 0,sin(phi),cos(phi)];
            R_y_theta = [cos(theta),0,sin(theta); 0,1,0; -sin(theta),0,cos(theta)];
            R_z_psi = [cos(psi),-sin(psi),0; sin(psi),cos(psi),0; 0,0,1];
            R_body_to_inertial = R_z_psi * R_y_theta * R_x_phi;
            
            F_gravity_inertial = [0; 0; m*g]; % NED: Z down is positive gravity
            F_thrust_body = [0; 0; -F_thrust_magnitude]; % FRD: -Z_body is up
            F_thrust_inertial = R_body_to_inertial * F_thrust_body;
            F_total_inertial = F_gravity_inertial + F_thrust_inertial;
            acc_linear_inertial = F_total_inertial / m;
            
            vel_new = vel_inertial + acc_linear_inertial * dt;
            pos_new = pos_inertial + vel_inertial * dt + 0.5 * acc_linear_inertial * dt^2;
            
            omega_cross_I_omega = cross(ang_vel_body, I * ang_vel_body);
            ang_acc_body = I \ (M_body - omega_cross_I_omega);
            ang_vel_body_new = ang_vel_body + ang_acc_body * dt;
            
            W_eul_rate_transform = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
                                    0, cos(phi),            -sin(phi);
                                    0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
            eul_rates = W_eul_rate_transform * ang_vel_body;
            eul_new = eul_angles + eul_rates * dt;
            % Optional: Normalize yaw to -pi to pi
            % eul_new(3) = atan2(sin(eul_new(3)), cos(eul_new(3))); 
        end
    end
end