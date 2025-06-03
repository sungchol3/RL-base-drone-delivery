classdef DroneRLEnvironment < rl.env.MATLABEnvironment
    % DroneRLEnvironment: 사용자 정의 강화학습 환경 클래스.
    % DroneSimulator를 사용하여 SAC 에이전트와 상호작용합니다.

    % 필수 속성 (RL Toolbox에서 사용)
    
    properties
        % 행동 명세 (ActionInfo)와 관찰 명세 (ObservationInfo)를 여기에 지정합니다.
        % ObservationInfo rl.util.rlNumericSpec
        % ActionInfo      rl.util.rlNumericSpec
        
        % 환경 상태를 저장하는 속성 (선택적)
        State % 현재 관찰값 (obs)
    end

    % 사용자 정의 속성
    properties (Access = public)
        DroneSim                % DroneSimulator 객체 인스턴스
        RewardParams            % 보상 함수 파라미터 구조체
        ActionScaling           % 행동 스케일링 파라미터 구조체
                                % .max_thrust, .max_torque_xy, .max_torque_z
        
        MissionWaypointsNED     % 전체 미션 웨이포인트 목록 [N,E,D; ...] (m)
        CurrentWaypointIndex    % 현재 목표 웨이포인트 인덱스
        CurrentTargetWaypointNED % 현재 목표 웨이포인트 [N;E;D] (m)
        
        MaxStepsPerEpisode      % 에피소드당 최대 스텝 수
        CurrentStep             % 현재 에피소드 내 스텝 수
        
        % (선택적) 진행 상황 보상을 위한 이전 거리
        PreviousDistanceToTarget 
    end

    methods
        % --- 생성자 ---
        function this = DroneRLEnvironment(droneSimulatorParams, missionWaypointsNED, rewardParams, actionScalingParams, rlEnvParams)
            % 입력:
            %   droneSimulatorParams: DroneSimulator 생성자용 파라미터 구조체
            %   missionWaypointsNED:  미션 웨이포인트 [Nx3] 매트릭스 (N,E,D 좌표)
            %   rewardParams:         calculate_reward 함수용 파라미터 구조체
            %   actionScalingParams:  행동 스케일링용 파라미터 구조체
            %                         (.max_thrust, .max_torque_xy, .max_torque_z)
            %   rlEnvParams:          RL 환경 관련 파라미터 구조체
            %                         (.max_steps_per_episode)

            % --- 관찰 공간 명세 (ObservationInfo) 정의 ---
            numObs = 12;
            pos_lm = ones(1,3) * norm(missionWaypointsNED) * 10; % Maximum distance : between waypoints * 10
            vel_lm = ones(1,3)*1e2; % Maximum speed : 100 (10m/0.1s)
            eular_ang_up = [pi/2, pi/2, 2*pi];
            eular_ang_lw = -[-pi/2, -pi/2, 0];
            ang_vel_lm = ones(1,3)*2*pi*5; % Maximum angular speed : 5 rev/s
            % obsLimits = ones(numObs, 1) * Inf; 
            % obsLimits = [...]; % 실제 예상 범위로 제한하는 것이 좋음
            obsInfo = rlNumericSpec([numObs 1], ...
                'LowerLimit', -[pos_lm, vel_lm, eular_ang_lw, ang_vel_lm]', ...
                'UpperLimit', [pos_lm, vel_lm, eular_ang_up, ang_vel_lm]');
            obsInfo.Name = 'Drone States and Errors';
            obsInfo.Description = 'Error(N,E,D), Vel(N,E,D), Att(R,P,Y), AngVel(p,q,r)';

            % --- 행동 공간 명세 (ActionInfo) 정의 ---
            numAct = 4;
            actInfo = rlNumericSpec([numAct 1], ...
                'LowerLimit', -1, ...
                'UpperLimit',  1);
            actInfo.Name = 'Normalized Drone Controls';
            actInfo.Description = 'Normalized Thrust, Mx, My, Mz';
            
            % === 상위 클래스 생성자 호출 ===
            disp('DroneRLEnvironment 생성자: 상위 클래스 생성자 호출 시도...');
            this = this@rl.env.MATLABEnvironment(obsInfo, actInfo); % 지역 변수 localObsInfo, localActInfo 사용
            disp('DroneRLEnvironment 생성자: 상위 클래스 생성자 호출 성공!');
            if isprop(this,'ObservationInfo') && isprop(this, 'ActionInfo')
                fprintf("ObservationInfo와 ActionInfo가 정상적으로 정의되었음\n");
            end
            

            % DroneSimulator 객체 생성
            % droneSimulatorParams는 drone_spec, initial_pose_xyz_rpy, flight_params, enable_visualization 등을 포함해야 함
            this.DroneSim = DroneSimulator(droneSimulatorParams.drone_spec, ...
                                           droneSimulatorParams.mission_waypoints_visualization, ... % 시각화용 웨이포인트
                                           droneSimulatorParams.initial_pose_xyz_rpy, ...
                                           droneSimulatorParams.flight_params, ...
                                           droneSimulatorParams.enable_visualization, ...
                                           droneSimulatorParams.video_options); % 비디오 옵션 추가

            this.MissionWaypointsNED = missionWaypointsNED;
            this.RewardParams = rewardParams;
            this.ActionScaling = actionScalingParams;
            this.MaxStepsPerEpisode = rlEnvParams.max_steps_per_episode;
            %{
            % --- 관찰 공간 명세 (ObservationInfo) ---
            % getObservation이 반환하는 12개 요소에 대한 명세
            % 각 요소의 예상 범위를 지정하는 것이 좋으나, 여기서는 일반적인 형태로 정의
            % 실제 값의 범위에 따라 LowerLimit/UpperLimit 조정 필요 (정규화에 중요)
            numObs = 12; % [err_N,err_E,err_D, vel_N,vel_E,vel_D, R,P,Y, p,q,r]
            obsLimits = ones(numObs, 1) * Inf; % 예시: 무한대 범위
            % 예시: 좀 더 구체적인 제한 (튜닝 필요)
            % obsLimits = [
            %     50; 50; 20; % max_error_N, E, D (m)
            %     10; 10; 5;  % max_vel_N, E, D (m/s)
            %     pi; pi; pi; % max_roll, pitch, yaw (rad)
            %     2*pi; 2*pi; 2*pi % max_p, q, r (rad/s)
            % ];
            this.ObservationInfo = rlNumericSpec([numObs 1], ...
                'LowerLimit', -obsLimits, ...
                'UpperLimit', obsLimits);
            this.ObservationInfo.Name = 'Drone States and Errors';
            this.ObservationInfo.Description = 'Error(N,E,D), Vel(N,E,D), Att(R,P,Y), AngVel(p,q,r)';

            % --- 행동 공간 명세 (ActionInfo) ---
            % SAC는 연속 행동 공간을 사용. 에이전트는 정규화된 값(예: [-1, 1])을 출력.
            % 4개 행동: [norm_F_thrust; norm_Mx; norm_My; norm_Mz]
            numAct = 4;
            this.ActionInfo = rlNumericSpec([numAct 1], ...
                'LowerLimit', -1, ...
                'UpperLimit',  1);
            this.ActionInfo.Name = 'Normalized Drone Controls';
            this.ActionInfo.Description = 'Normalized Thrust, Mx, My, Mz';
            %}
            % 초기 상태 업데이트 (reset 함수에서 수행)
            % updateActionInfo(this); % ActionInfo가 상태에 따라 변하는 경우
            this.State = zeros(numObs, 1); 
            disp('DroneRLEnvironment 생성자: 사용자 정의 속성 초기화 완료.');
            
            disp('<<<<< DroneRLEnvironment 생성자: 성공적으로 완료 및 종료됨 >>>>>');
        end

        % --- 에피소드 시작 시 환경 리셋 ---
        function initialObservation = reset(this)
            % DroneSimulator 리셋
            this.DroneSim.reset();
            
            % 미션 웨이포인트 관련 변수 초기화
            this.CurrentWaypointIndex = 1; % 첫 번째 웨이포인트가 시작점, 두 번째부터 목표
            if size(this.MissionWaypointsNED, 1) < this.CurrentWaypointIndex + 1
                error('미션 웨이포인트가 최소 2개 이상이어야 합니다 (시작점 + 첫 목표).');
            end
            % 첫 번째 목표 웨이포인트 설정
            this.CurrentTargetWaypointNED = this.MissionWaypointsNED(this.CurrentWaypointIndex + 1, :)';
            
            % 시뮬레이션 스텝 카운터 초기화
            this.CurrentStep = 0;
            
            % (선택적) 진행 상황 보상을 위한 이전 거리 초기화
            if isfield(this.RewardParams, 'w_progress') && this.RewardParams.w_progress > 0
                initial_pos_err_vec = this.CurrentTargetWaypointNED - this.DroneSim.CurrentState.pos_inertial;
                this.PreviousDistanceToTarget = norm(initial_pos_err_vec);
            else
                this.PreviousDistanceToTarget = []; % 사용 안 함
            end

            % 초기 관찰값 계산 및 반환
            observation = this.DroneSim.getObservation(this.CurrentTargetWaypointNED);
            this.State = observation; % 내부 상태 업데이트
            initialObservation = this.State;
            
            % (디버깅용)
            % disp('환경 리셋 완료. 초기 목표 웨이포인트 (NED):');
            % disp(this.CurrentTargetWaypointNED');
        end

        %% --- 한 스텝 진행 ---
        function [observation, reward, isDone, loggedSignals] = step(this, action_normalized)
            loggedSignals = []; % 추가 정보 로깅용 (비워둠)
            this.CurrentStep = this.CurrentStep + 1;

            % 1. 에이전트의 정규화된 행동(action_normalized)을 실제 물리적 제어 입력으로 스케일링
            control_inputs.F_thrust = ((action_normalized(1) + 1) / 2) * this.ActionScaling.max_thrust; % [0, max_thrust]
            control_inputs.M_body = [
                action_normalized(2) * this.ActionScaling.max_torque_xy;  % [-max_torque_xy, max_torque_xy]
                action_normalized(3) * this.ActionScaling.max_torque_xy;  % [-max_torque_xy, max_torque_xy]
                action_normalized(4) * this.ActionScaling.max_torque_z   % [-max_torque_z, max_torque_z]
            ];
            
            % (선택적) 제어 입력 클리핑 (만약 에이전트 출력이 정확히 [-1,1]을 벗어날 수 있다면)
            control_inputs.F_thrust = max(0, min(control_inputs.F_thrust, this.ActionScaling.max_thrust));
            control_inputs.M_body(1) = max(-this.ActionScaling.max_torque_xy, min(control_inputs.M_body(1), this.ActionScaling.max_torque_xy));
            control_inputs.M_body(2) = max(-this.ActionScaling.max_torque_xy, min(control_inputs.M_body(2), this.ActionScaling.max_torque_xy));
            control_inputs.M_body(3) = max(-this.ActionScaling.max_torque_z,  min(control_inputs.M_body(3), this.ActionScaling.max_torque_z));


            % 2. DroneSimulator 한 스텝 진행
            % DroneSim.step은 다음 상태와 '실제' 선형 가속도를 반환할 수 있음 (보상 계산에 사용 가능)
            [next_drone_physical_state, ~] = this.DroneSim.step(control_inputs);
            % next_drone_physical_state는 this.DroneSim.CurrentState와 동일한 내용임 (step 내부에서 업데이트됨)

            % 3. 다음 관찰(Observation) 계산
            observation = this.DroneSim.getObservation(this.CurrentTargetWaypointNED);
            this.State = observation; % 내부 상태 업데이트

            % 4. 에피소드 종료 조건(isDone) 판단
            isDone = false;
            was_successful_termination = false;
            was_crash_termination = false;

            current_pos_NED = next_drone_physical_state.pos_inertial;
            current_altitude = -current_pos_NED(3); % 고도 (Up is positive)
            
            % 가. 목표 웨이포인트 도달 확인
            dist_to_target = norm(current_pos_NED - this.CurrentTargetWaypointNED);
            if dist_to_target < this.RewardParams.arrival_threshold_dist
                was_successful_termination = true;
                % 마지막 웨이포인트인지 확인
                if this.CurrentWaypointIndex + 1 >= size(this.MissionWaypointsNED, 1)
                    isDone = true; % 모든 웨이포인트 도달
                    disp('모든 목표 웨이포인트 도달! 에피소드 성공 종료.');
                else
                    % 다음 웨이포인트로 목표 변경 (에피소드는 계속)
                    this.CurrentWaypointIndex = this.CurrentWaypointIndex + 1;
                    this.CurrentTargetWaypointNED = this.MissionWaypointsNED(this.CurrentWaypointIndex + 1, :)';
                    disp(['웨이포인트 ', num2str(this.CurrentWaypointIndex),' 도달. 다음 목표: (NED) ', num2str(this.CurrentTargetWaypointNED')]);
                    % (선택적) 진행 상황 보상을 위한 이전 거리 업데이트
                    if isfield(this.RewardParams, 'w_progress') && this.RewardParams.w_progress > 0
                        new_pos_err_vec = this.CurrentTargetWaypointNED - current_pos_NED;
                        this.PreviousDistanceToTarget = norm(new_pos_err_vec);
                    end
                end
            end
            
            % 나. 충돌 조건 (예: 고도가 너무 낮거나, 과도한 기울임)
            if current_altitude < 0.1 % 지면 충돌 (예시 임계값)
                isDone = true;
                was_crash_termination = true;
                disp('지면 충돌! 에피소드 실패 종료.');
            end
            
            max_roll_pitch = this.RewardParams.max_allowable_roll_pitch; % 예: deg2rad(80)
            if abs(next_drone_physical_state.eul_angles(1)) > max_roll_pitch || ...
               abs(next_drone_physical_state.eul_angles(2)) > max_roll_pitch
                isDone = true;
                was_crash_termination = true;
                disp('과도한 기울임! 에피소드 실패 종료.');
            end

            % 다. 최대 스텝 수 초과
            if this.CurrentStep >= this.MaxStepsPerEpisode
                isDone = true;
                if ~was_successful_termination % 목표 도달 못하고 시간 초과
                    disp('최대 스텝 수 초과. 에피소드 종료.');
                end
            end

            % 5. 보상(Reward) 계산
            % (선택적) 진행 상황 보상을 위해 previous_distance_to_target 전달
            reward_params_for_calc = this.RewardParams;
            if isfield(reward_params_for_calc, 'w_progress') && reward_params_for_calc.w_progress > 0
                reward_params_for_calc.previous_distance_to_target = this.PreviousDistanceToTarget;
            end
            
            reward = calculate_reward(next_drone_physical_state, control_inputs, ...
                                      this.CurrentTargetWaypointNED, ...
                                      isDone, was_successful_termination, was_crash_termination, ...
                                      reward_params_for_calc);
            
            % (선택적) 진행 상황 보상을 위한 이전 거리 업데이트 (다음 스텝용)
            if isfield(this.RewardParams, 'w_progress') && this.RewardParams.w_progress > 0 && ~isDone
                this.PreviousDistanceToTarget = dist_to_target;
            end
        end % step 끝
        
        % (선택적) 시각화 메서드 (RL Toolbox에서 직접 사용하진 않지만, 테스트용)
        % function render(this)
        %     if this.DroneSim.EnableVisualization
        %         this.DroneSim.updateVisualization();
        %     end
        % end
    end

    methods (Access = public)
        function obsInfo = getObservationInfo(this)
            % ObservationInfo 속성값을 반환하는 공개 Getter 메서드
            obsInfo = this.ObservationInfo; % 클래스 내부에서는 protected 속성 접근 가능
        end
        
        function actInfo = getActionInfo(this)
            % ActionInfo 속성값을 반환하는 공개 Getter 메서드
            actInfo = this.ActionInfo; % 클래스 내부에서는 protected 속성 접근 가능
        end
    end
end