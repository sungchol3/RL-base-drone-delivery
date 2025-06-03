function reward = calculate_reward(current_drone_state, applied_action, current_target_NED, ...
                                   is_episode_done, was_successful_termination, was_crash_termination, ...
                                   reward_params)
% calculate_reward: 드론의 현재 상태, 행동, 목표 등을 바탕으로 보상을 계산합니다.
%
% 입력:
%   current_drone_state: DroneSimulator의 현재 상태 구조체
%       .pos_inertial: [N; E; D] (m) - 관성 좌표계 기준 위치
%       .vel_inertial: [Vn; Ve; Vd] (m/s) - 관성 좌표계 기준 속도
%       .eul_angles:   [Roll; Pitch; Yaw] (rad) - 오일러 각
%       .ang_vel_body: [p; q; r] (rad/s) - 동체 기준 각속도
%   applied_action: RL 에이전트가 취한 후 스케일링된 실제 제어 입력 구조체
%       .F_thrust: (scalar) 적용된 총 추력 (N)
%       .M_body:   [3x1] 적용된 동체 기준 토크 [Mx; My; Mz] (Nm)
%   current_target_NED: [3x1] 현재 목표 웨이포인트 [N_target; E_target; D_target] (m)
%   is_episode_done: (boolean) 이번 스텝에서 에피소드가 종료되었는지 여부
%   was_successful_termination: (boolean) 목표 도달 등으로 성공적으로 종료되었는지 여부
%   was_crash_termination: (boolean) 충돌 또는 경계 이탈 등으로 실패하며 종료되었는지 여부
%   reward_params: 보상 계산에 필요한 파라미터 및 가중치 구조체
%       .w_time_penalty:         스텝당 시간 벌점 가중치 (양수)
%       .w_distance_penalty:     목표까지의 거리에 대한 벌점 가중치 (양수)
%       .w_action_thrust_effort: 추력 사용량(호버링 대비)에 대한 벌점 가중치 (양수)
%       .w_action_torque_effort: 토크 사용량에 대한 벌점 가중치 (양수)
%       .w_ang_vel_penalty:      과도한 각속도에 대한 벌점 가중치 (양수)
%       .w_attitude_penalty:     과도한 기울임(롤/피치)에 대한 벌점 가중치 (양수)
%       .bonus_reach_target:     목표 도달 시 큰 양의 보너스
%       .penalty_crash:          충돌 시 큰 음의 벌점 (음수 값)
%       .penalty_timeout:        시간 초과로 목표 미도달 시 벌점 (음수 값, 선택적)
%       .hover_thrust:           호버링에 필요한 기본 추력 (N)
%       .arrival_threshold_dist: 목표 도달로 간주하는 거리 임계값 (m)
%       .max_allowable_roll_pitch: 허용 가능한 최대 롤/피치 각도 (rad)
%       .progress_weight:        (선택적) 목표에 가까워지는 것에 대한 보상 가중치
%
% 출력:
%   reward: (scalar) 이번 스텝에 대한 총 보상 값

    % --- 1. 기본 스텝 보상 (시간 경과에 대한 벌점) ---
    current_reward = -reward_params.w_time_penalty;

    % --- 2. 목표 지점과의 거리 기반 보상/벌점 ---
    pos_err_vec = current_target_NED - current_drone_state.pos_inertial;
    distance_to_target = norm(pos_err_vec);

    % 목표 지점과의 거리에 대한 벌점 (항상 목표를 향하도록 유도)
    current_reward = current_reward - reward_params.w_distance_penalty * distance_to_target;
    
    % (선택적) 목표 지점에 가까워지는 '진행(progress)'에 대한 보상
    % 이를 위해서는 이전 스텝의 distance_to_target 값이 필요함.
    % if isfield(reward_params, 'previous_distance_to_target') && ~isempty(reward_params.previous_distance_to_target)
    %     progress = reward_params.previous_distance_to_target - distance_to_target;
    %     current_reward = current_reward + reward_params.w_progress * progress;
    % end


    % --- 3. 안정성 및 제어 노력 관련 벌점 ---
    % الف. 제어 입력(행동) 크기에 대한 벌점 (부드러운 제어 유도)
    thrust_effort_penalty = (applied_action.F_thrust - reward_params.hover_thrust)^2;
    torque_effort_penalty = sum(applied_action.M_body.^2); % 각 토크 요소 제곱의 합
    current_reward = current_reward - reward_params.w_action_thrust_effort * thrust_effort_penalty;
    current_reward = current_reward - reward_params.w_action_torque_effort * torque_effort_penalty;

    % ب. 과도한 각속도에 대한 벌점 (안정적인 자세 유지 유도)
    angular_velocity_penalty = sum(current_drone_state.ang_vel_body.^2);
    current_reward = current_reward - reward_params.w_ang_vel_penalty * angular_velocity_penalty;

    % ج. 과도한 기울임(롤/피치 각도)에 대한 벌점
    roll_angle  = current_drone_state.eul_angles(1);
    pitch_angle = current_drone_state.eul_angles(2);
    
    % 지정된 허용 각도를 넘어서는 것에 대해 벌점을 주거나, 단순히 각도 자체에 벌점
    attitude_penalty = 0;
    if abs(roll_angle) > reward_params.max_allowable_roll_pitch
        attitude_penalty = attitude_penalty + (abs(roll_angle) - reward_params.max_allowable_roll_pitch)^2;
    end
    if abs(pitch_angle) > reward_params.max_allowable_roll_pitch
        attitude_penalty = attitude_penalty + (abs(pitch_angle) - reward_params.max_allowable_roll_pitch)^2;
    end
    % 또는 더 간단하게:
    % attitude_penalty = roll_angle^2 + pitch_angle^2;
    current_reward = current_reward - reward_params.w_attitude_penalty * attitude_penalty;
    

    % --- 4. 에피소드 종료 시 보상/벌점 ---
    if is_episode_done
        if was_successful_termination % 목표 지점 도달 성공
            current_reward = current_reward + reward_params.bonus_reach_target;
            
            % (선택적) 목표 도달 시 정확도에 따른 추가 보너스
            % 예: 거리가 가까울수록 더 큰 보너스 (이미 위에서 거리 벌점이 있지만, 성공 시 추가 보상 가능)
            % current_reward = current_reward + (reward_params.arrival_threshold_dist / (distance_to_target + 1e-6)) * some_accuracy_bonus_factor;

        elseif was_crash_termination % 충돌 또는 경계 이탈
            current_reward = current_reward + reward_params.penalty_crash; % penalty_crash는 큰 음수 값
        else % 시간 초과 등으로 목표 미도달
            if isfield(reward_params, 'penalty_timeout') && ~isempty(reward_params.penalty_timeout)
                current_reward = current_reward + reward_params.penalty_timeout;
            end
        end
    end
    
    reward = current_reward;
end