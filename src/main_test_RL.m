% =========================================================================
% Main Script for Drone SAC Reinforcement Learning Training and Visualization
% =========================================================================
clear all; close all; clc;
clear classes; rehash toolboxcache;

addpath("src/environment/");

disp('강화학습 드론 제어 시뮬레이션 시작...');

%% --- 1. 시뮬레이션, 보상, RL 하이퍼파라미터 설정 ---
disp('파라미터 설정 중...');

% 가. DroneSimulator용 파라미터 (droneSimParams)
drone_spec.drone_mass = 30.0; % kg
drone_spec.payload_mass = 1.0; % kg
drone_spec.inertia = diag([0.6, 0.6, 1.2]); % 예시 관성 모멘트 (Ix, Iy, Iz in kg*m^2) - 튜닝 필요!

initial_pose_xyz_rpy = [-10, 0, 10, 0, 0, 0]; % 시각화용 좌표계 (X, Y, Z_altitude), (Roll, Pitch, Yaw in rad)

% 미션 웨이포인트 (시각화용 좌표계: X, Y, Z_altitude)
% 예: 시작점 -> 중간점 -> 최종점
mission_waypoints = [
    initial_pose_xyz_rpy(1), initial_pose_xyz_rpy(2), initial_pose_xyz_rpy(3); % 시작점
    0,  0,  0    % 최종 목표 (착륙)
];

flight_params.update_rate = 50; % Hz, 시뮬레이션 및 제어 주기 (1/dt)

video_options.enable = false; % 비디오 녹화 비활성화 (필요시 true로 변경)
video_options.filename = 'sac_trained_flight.mp4';
video_options.framerate = 10; % DroneSimulator의 시각화 업데이트 빈도와 맞출 것

droneSimParams.drone_spec = drone_spec;
droneSimParams.mission_waypoints_visualization = mission_waypoints;
droneSimParams.initial_pose_xyz_rpy = initial_pose_xyz_rpy;
droneSimParams.flight_params = flight_params;
droneSimParams.enable_visualization = false; % 학습 중에는 보통 false, 평가 시 true
droneSimParams.video_options = video_options;

% 나. 미션 웨이포인트 (NED 좌표계: North, East, Down)
% 시각화용 X,Y,Alt -> NED (N=X, E=Y, D=-Alt)
missionWaypointsNED = zeros(size(mission_waypoints));
missionWaypointsNED(:,1) = mission_waypoints(:,1); % N = X
missionWaypointsNED(:,2) = mission_waypoints(:,2); % E = Y
missionWaypointsNED(:,3) = mission_waypoints(:,3);% D = -Altitude

% 다. 보상 함수 파라미터 (rewardParams)
rewardParams.w_time_penalty         = 0.05;
rewardParams.w_distance_penalty     = 0.1;
rewardParams.w_action_thrust_effort = 0.00005; % 추력 변화량에 대한 패널티로 변경 고려
rewardParams.w_action_torque_effort = 0.0005;
rewardParams.w_ang_vel_penalty      = 0.005;
rewardParams.w_attitude_penalty     = 0.02;
rewardParams.bonus_reach_target     = 500;  % 각 웨이포인트 도달 시
rewardParams.bonus_final_target     = 1000; % 최종 목표 도달 시 추가 보너스
rewardParams.penalty_crash          = -500;
rewardParams.penalty_timeout        = -50;
rewardParams.hover_thrust           = (drone_spec.drone_mass + drone_spec.payload_mass) * 9.81;
rewardParams.arrival_threshold_dist = 1.5; % m, 웨이포인트 도달 임계값
rewardParams.max_allowable_roll_pitch = deg2rad(45); % rad
rewardParams.w_progress             = 0.2; % 목표에 가까워지는 것에 대한 보상 가중치

% 라. SAC 에이전트 및 학습 하이퍼파라미터 (rlHyperParams)
rlHyperParams.actorLR = 1e-4;
rlHyperParams.criticLR = 1e-3;
rlHyperParams.discountFactor = 0.99;
rlHyperParams.experienceBufferLength = 1e6;
rlHyperParams.miniBatchSize = 256; % 128 또는 256
rlHyperParams.targetSmoothFactor = 5e-3; % Tau for soft target updates
rlHyperParams.numWarmupSteps = 2000; % 학습 시작 전 랜덤 행동 스텝 수 (환경 스텝 기준)
% 액터 신경망의 평균 및 표준편차 출력 레이어 이름
rlHyperParams.actorMeanOutputLayerName = 'actor_mean_tanh'; % createActorNetwork에서 설정한 실제 이름 사용
rlHyperParams.actorStdOutputLayerName = 'actor_std_softplus'; % createActorNetwork에서 설정한 실제 이름 사용

% 크리틱 신경망의 관찰 및 행동 입력 레이어 이름
rlHyperParams.criticObsInputName = 'observation'; % createCriticNetwork에서 설정한 실제 이름 사용
rlHyperParams.criticActionInputName = 'action';   % createCriticNetwork에서 설정한 실제 이름 사용

% 신경망 구조 정의 (SACTrainingManager 생성자에서 obsInfo, actInfo를 받아야 정확한 차원 설정 가능)
% 임시로 차원 지정, SACTrainingManager 내부에서 실제 환경의 obsInfo, actInfo 사용하도록 수정 필요
obsDimTemp = 12; % 예시 관찰 차원
actDimTemp = 4;  % 예시 행동 차원

rlHyperParams.actorNetworkArch.commonLayers = [
    fullyConnectedLayer(256, 'Name', 'actor_fc1')
    reluLayer('Name', 'actor_relu1')
    fullyConnectedLayer(256, 'Name', 'actor_fc2') % 레이어 추가
    reluLayer('Name', 'actor_relu2') 
];
rlHyperParams.actorNetworkArch.meanLayers = [
    fullyConnectedLayer(actDimTemp, 'Name', 'actor_mean_output') 
    tanhLayer('Name','actor_mean_tanh') % 최종 평균 출력 레이어 이름: actor_mean_tanh
];
rlHyperParams.actorNetworkArch.stdLayers = [
    fullyConnectedLayer(actDimTemp, 'Name', 'actor_std_output') 
    softplusLayer('Name','actor_std_softplus') % 최종 표준편차 출력 레이어 이름: actor_std_softplus
];
% 그리고 SACTrainingManager 생성 시 rlHyperParams에 아래처럼 레이어 이름을 전달
rlHyperParams.actorMeanOutputLayerName = 'actor_mean_tanh';
rlHyperParams.actorStdOutputLayerName = 'actor_std_softplus';

rlHyperParams.criticNetworkArch.statePath = [
    fullyConnectedLayer(128, 'Name', 'critic_state_fc1') % 노드 수 증가
    reluLayer('Name', 'critic_state_relu1')
];
rlHyperParams.criticNetworkArch.actionPath = [
    fullyConnectedLayer(128, 'Name', 'critic_action_fc1') % 노드 수 증가
    reluLayer('Name', 'critic_action_relu1')
];
rlHyperParams.criticNetworkArch.commonPath = [
    fullyConnectedLayer(256, 'Name', 'critic_common_fc1')
    reluLayer('Name', 'critic_common_relu1')
    fullyConnectedLayer(1, 'Name', 'q_value_output')
];

rlHyperParams.targetEntropy = 'auto'; % 또는 -actDimTemp (행동 공간 차원의 음수값)
rlHyperParams.alphaLearningRate = 1e-4; 

rlHyperParams.maxEpisodes = 5000; % 학습 에피소드 수
rlHyperParams.maxStepsPerEpisode = ceil(200 / (1/flight_params.update_rate)); % 예: 최대 200초 비행
rlHyperParams.scoreAveragingWindowLength = 50;
rlHyperParams.stopTrainingCriteria = "AverageReward";
rlHyperParams.stopTrainingValue = 20000; % 목표 평균 보상 (문제와 보상 스케일에 따라 크게 다름)
rlHyperParams.saveAgentCriteria = "EpisodeReward"; % 또는 "AverageReward"
rlHyperParams.saveAgentValue = 18000; % 에이전트 저장 조건 값
rlHyperParams.saveAgentPath = 'saved_drone_sac_agents';
rlHyperParams.verbose = true;
rlHyperParams.plots = "training-progress"; % 학습 중 플롯 표시
rlHyperParams.useParallel = true; % 병렬 학습 사용 여부

% 행동 스케일링 파라미터 (DroneRLEnvironment로 전달됨)
rlHyperParams.actionScaling.max_thrust = 2.0 * rewardParams.hover_thrust; % 최대 추력
rlHyperParams.actionScaling.max_torque_xy = 30;  % Nm, 튜닝 필요
rlHyperParams.actionScaling.max_torque_z  = 10;   % Nm, 튜닝 필요

disp('파라미터 설정 완료.');

%% --- 2. SACTrainingManager 객체 생성 ---
disp('SACTrainingManager 객체 생성 중...');
try
    trainer = SACTrainingManager(droneSimParams, missionWaypointsNED, rewardParams, rlHyperParams);
    disp('SACTrainingManager 객체 생성 완료.');
catch ME
    disp('SACTrainingManager 객체 생성 실패:');
    rethrow(ME);
end

%% --- 3. 에이전트 학습 실행 ---
disp('SAC 에이전트 학습 시작...');
try
    [trainedAgent, trainingStats] = trainer.trainAgent();
    disp('SAC 에이전트 학습 완료!');
    
    % 학습 결과 플로팅 (선택적)
    if ~isempty(trainingStats) && isprop(trainingStats, 'EpisodeReward')
        figure;
        plot(trainingStats.EpisodeReward);
        xlabel('에피소드');
        ylabel('누적 보상');
        title('SAC 에이전트 학습 진행 상황');
        grid on;
    end
    
catch ME
    disp('에이전트 학습 중 오류 발생:');
    rethrow(ME);
end

%% 학습결과 플로팅
figure('Name',"Training Result")
plot(trainingStats.EpisodeReward);
xlabel('Episod');
ylabel('Cumulative Reward')
title("SAC Agent Train Process")
grid on

%% --- 4. 학습된 에이전트로 시뮬레이션 및 경로 시각화 ---
if exist('trainedAgent', 'var') && ~isempty(trainedAgent)
    disp('학습된 에이전트로 평가 시뮬레이션 시작...');
    
    % 시뮬레이션용 환경의 시각화 옵션 활성화
    trainer.RLEnvironment.DroneSim.EnableVisualization = true; 
    % 비디오 녹화를 원하면 여기서 video_options도 true로 설정하고 SACTrainingManager가 이를 반영하도록 수정 필요
    % trainer.RLEnvironment.DroneSim.EnableVideoRecording = true; % 예시
    % trainer.RLEnvironment.DroneSim.VideoObject = VideoWriter(...) % 재설정 필요할 수 있음

    % SACTrainingManager에 평가용 메서드가 있다면 사용, 없다면 직접 루프 구성
    % 예시: SACTrainingManager에 runSingleEpisodeWithAgent 메서드가 있다고 가정
    if ismethod(trainer, 'runSingleEpisodeWithAgent')
        num_eval_episodes = 10; % 평가 에피소드 수
        all_trajectories = cell(num_eval_episodes, 1);
        for ep = 1:num_eval_episodes
            fprintf('평가 에피소드 %d 시작...\n', ep);
            % runSingleEpisodeWithAgent는 DroneSimulator의 TrajectoryLog를 반환하도록 수정 필요
            % 또는, DroneRLEnvironment의 loggedSignals를 통해 trajectory를 받아오도록 설계
            % 여기서는 SACTrainingManager가 내부적으로 DroneSimulator의 로그를 가져온다고 가정
            
            % runSingleEpisodeWithAgent 메서드가 DroneSimulator의 로그를 반환하도록 수정했다고 가정
             eval_trajectory_data = trainer.runSingleEpisodeWithAgent(trainedAgent, true); 
             all_trajectories{ep} = eval_trajectory_data;
            fprintf('평가 에피소드 %d 완료.\n', ep);
        end
        
        % 첫 번째 평가 에피소드의 경로 시각화
        if ~isempty(all_trajectories{1}) && isfield(all_trajectories{1}, 'PositionNED')
            trajectory_to_plot = all_trajectories{1};
            
            figure;
            plot3(trajectory_to_plot.PositionNED(:,2), ... % East (Y축에 해당)
                  trajectory_to_plot.PositionNED(:,1), ... % North (X축에 해당)
                  -trajectory_to_plot.PositionNED(:,3), ...% Up (-D)
                  'b-', 'LineWidth', 1.5);
            hold on;
            
            % 웨이포인트 플롯 (NED -> 시각화 좌표계 변환)
            plot3(missionWaypointsNED(:,2), missionWaypointsNED(:,1), missionWaypointsNED(:,3), ...
                  'ro-', 'MarkerFaceColor','r', 'MarkerSize', 8, 'DisplayName','미션 경로 (NED)');
            
            % 시작점 표시
            plot3(missionWaypointsNED(1,2), missionWaypointsNED(1,1), missionWaypointsNED(1,3), ...
                  'g^', 'MarkerFaceColor','g', 'MarkerSize', 10, 'DisplayName','시작점');
            % 최종 목표점 표시
            plot3(missionWaypointsNED(end,2), missionWaypointsNED(end,1), missionWaypointsNED(end,3), ...
                  'ks', 'MarkerFaceColor','k', 'MarkerSize', 10, 'DisplayName','최종 목표점');

            xlabel('East (m)');
            ylabel('North (m)');
            zlabel('Altitude (Up) (m)');
            title('학습된 SAC 에이전트의 비행 경로');
            legend('show', 'Location','best');
            grid on;
            axis equal;
            view(30, 20); % 3D 뷰 각도 조절
            hold off;
            
            % Impulse
            vel = trajectory_to_plot.Vel;
            diff_v = diff(vel(end-9:end,:));
            impulse = 31 * mean(diff_v, 1);
            fprintf("Impulse : %.2f(N*m)\n",impulse);
        else
            disp('시뮬레이션 결과에서 경로 데이터를 찾을 수 없습니다.');
        end

        % Total flight time
        totalFlightTime_mean = cellfun(@(t) mean(t.Time), all_trajectories);
        fprintf("Mean of totalFlightTime of test : %.1f (s)\n", mean(totalFlightTime_mean));

    else
        disp('runSingleEpisodeWithAgent 메서드를 SACTrainingManager에서 찾을 수 없습니다.');
    end
else
    disp('학습된 에이전트가 없습니다. 시뮬레이션을 건너<0xEB><0><0xA4>니다.');
end

%% Plot another mission
trajectory_to_plot = all_trajectories{10};
            
figure;
plot3(trajectory_to_plot.PositionNED(:,2), ... % East (Y축에 해당)
      trajectory_to_plot.PositionNED(:,1), ... % North (X축에 해당)
      -trajectory_to_plot.PositionNED(:,3), ...% Up (-D)
      'b-', 'LineWidth', 1.5);
hold on;

% 웨이포인트 플롯 (NED -> 시각화 좌표계 변환)
plot3(missionWaypointsNED(:,2), missionWaypointsNED(:,1), missionWaypointsNED(:,3), ...
      'ro-', 'MarkerFaceColor','r', 'MarkerSize', 8, 'DisplayName','미션 경로 (NED)');

% 시작점 표시
plot3(missionWaypointsNED(1,2), missionWaypointsNED(1,1), missionWaypointsNED(1,3), ...
      'g^', 'MarkerFaceColor','g', 'MarkerSize', 10, 'DisplayName','시작점');
% 최종 목표점 표시
plot3(missionWaypointsNED(end,2), missionWaypointsNED(end,1), missionWaypointsNED(end,3), ...
      'ks', 'MarkerFaceColor','k', 'MarkerSize', 10, 'DisplayName','최종 목표점');

xlabel('East (m)');
ylabel('North (m)');
zlabel('Altitude (Up) (m)');
title('학습된 SAC 에이전트의 비행 경로');
legend('show', 'Location','best');
grid on;
axis equal;
view(30, 20); % 3D 뷰 각도 조절
hold off;

disp('메인 스크립트 실행 완료.');