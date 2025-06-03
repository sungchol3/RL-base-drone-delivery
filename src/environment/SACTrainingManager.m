classdef SACTrainingManager < handle
    % SACTrainingManager: SAC 에이전트 학습 과정을 관리하는 클래스.
    % DroneRLEnvironment를 사용하여 드론 제어 학습을 수행합니다.

    properties (SetAccess = private)
        DroneSimParams      % DroneSimulator 생성용 파라미터 구조체
        MissionWaypointsNED % 미션 웨이포인트 (NED)
        RewardParams        % 보상 함수 파라미터 구조체
        RLHyperParams       % SAC 에이전트 및 학습 관련 하이퍼파라미터 구조체
        
        RLEnvironment       % DroneRLEnvironment 객체 인스턴스
        
        ActorNetwork        % 액터 신경망 (dlnetwork)
        CriticNetwork1      % 첫 번째 크리틱 신경망 (dlnetwork)
        CriticNetwork2      % 두 번째 크리틱 신경망 (dlnetwork)
        
        agent            % rlSACAgent 객체
        
        TrainingOptions     % rlTrainingOptions 객체
        TrainingStats       % train 함수로부터 반환된 학습 통계
        
        TrainedAgentPath    % 학습된 에이전트를 저장할 기본 경로
    end

    methods
        % --- 생성자 ---
        function obj = SACTrainingManager(droneSimParams, missionWaypointsNED, rewardParams, rlHyperParams)
            % 입력:
            %   droneSimParams: DroneSimulator 생성자용 파라미터 구조체
            %   missionWaypointsNED: 미션 웨이포인트 [Nx3] (N,E,D 좌표)
            %   rewardParams: calculate_reward 함수용 파라미터 구조체
            %   rlHyperParams: SAC 에이전트 및 학습용 하이퍼파라미터 구조체
            
            disp('--- SACTrainingManager 객체 초기화 시작 ---');
            
            obj.DroneSimParams = droneSimParams;
            obj.MissionWaypointsNED = missionWaypointsNED;
            obj.RewardParams = rewardParams;
            obj.RLHyperParams = rlHyperParams;
            
            % 1. DroneRLEnvironment 객체 생성
            % rlEnvParams는 rlHyperParams에서 가져오거나 별도로 관리 가능
            rlEnvParams.max_steps_per_episode = rlHyperParams.maxStepsPerEpisode;
            obj.RLEnvironment = DroneRLEnvironment(droneSimParams, missionWaypointsNED, rewardParams, ...
                                                 rlHyperParams.actionScaling, rlEnvParams);
            
            obsInfo = obj.RLEnvironment.getObservationInfo();
            actInfo = obj.RLEnvironment.getActionInfo();

            % 2. 액터 및 크리틱 신경망 생성
            actorDlNetwork = obj.createActorNetwork(obsInfo, actInfo, rlHyperParams.actorNetworkArch);
            criticDlNetwork1 = obj.createCriticNetwork(obsInfo, actInfo, rlHyperParams.criticNetworkArch);
            criticDlNetwork2 = obj.createCriticNetwork(obsInfo, actInfo, rlHyperParams.criticNetworkArch);
            
            %%% action 정의
            actor = rlContinuousGaussianActor(actorDlNetwork, obsInfo, actInfo, ...
                'ActionMeanOutputNames', rlHyperParams.actorMeanOutputLayerName, ...
                'ActionStandardDeviationOutputNames', rlHyperParams.actorStdOutputLayerName);
             
            %%% criticDlNetwork들을 사용하여 rlQValueFunction 객체 생성
            critic1_rl_object = rlQValueFunction(criticDlNetwork1, obsInfo, actInfo, ...
                'ObservationInputNames', {rlHyperParams.criticObsInputName}, ...
                'ActionInputNames', {rlHyperParams.criticActionInputName});

            critic2_rl_object = rlQValueFunction(criticDlNetwork2, obsInfo, actInfo, ...
                'ObservationInputNames', rlHyperParams.criticObsInputName, ...
                'ActionInputNames', rlHyperParams.criticActionInputName);
            
            obs = rand(obsInfo.Dimension); act = rand(actInfo.Dimension) % test
            disp('QValue: '); disp(getValue(critic1_rl_object, {obs},{act}));
            obj.ActorNetwork = actor;     % rlContinuousGaussianActor 할당
            obj.CriticNetwork1 = critic1_rl_object; % rlQValueFunction 할당
            obj.CriticNetwork2 = critic2_rl_object; % rlQValueFunction 할당

            % 3. SAC 에이전트 옵션 설정 및 에이전트 생성
            agentOpts = rlSACAgentOptions(...
                'SampleTime', obj.RLEnvironment.DroneSim.TimeStep, ...
                'DiscountFactor', rlHyperParams.discountFactor, ...
                'ExperienceBufferLength', rlHyperParams.experienceBufferLength, ...
                'MiniBatchSize', rlHyperParams.miniBatchSize, ...
                'TargetSmoothFactor', rlHyperParams.targetSmoothFactor, ...
                'SaveExperienceBufferWithAgent', true); % 에이전트 저장 시 버퍼도 함께 저장

            % delete in options : 'NumWarmStartSteps', rlHyperParams.numWarmStartSteps, ...

            % --- 엔트로피 가중치 (alpha) 자동 튜닝 설정 수정 ---
            % 1. rlEntropyWeightOptions 객체 생성
            entropyOpts = EntropyWeightOptions();
            
            % 2. EntropyWeightOptions 객체에 관련 속성 설정
            if strcmpi(rlHyperParams.targetEntropy, 'auto')
                % TargetEntropy를 자동으로 설정 (일반적으로 행동 공간 차원의 음수 값)
                % actInfo.Dimension이 [numActions 1] 형태이므로 prod(actInfo.Dimension)은 numActions와 동일
                entropyOpts.TargetEntropy = -prod(actInfo.Dimension); 
                entropyOpts.LearnRate = rlHyperParams.alphaLearningRate; % 알파 학습률
                entropyOpts.EntropyWeight = true; % 알파 자동 튜닝 활성화 (기본값일 수 있음)
            else
                % TargetEntropy를 특정 숫자 값으로 지정하는 경우
                entropyOpts.TargetEntropy = rlHyperParams.targetEntropy; 
                entropyOpts.LearnRate = rlHyperParams.alphaLearningRate; % 알파 학습률
                entropyOpts.EntropyWeight = true; % 알파 자동 튜닝 활성화
                % 만약 알파 값을 고정하고 싶다면, agentOpts에서 직접 설정:
                % agentOpts.EntropyWeight = fixed_alpha_value; 
                % 그리고 entropyOpts.LearnEntropyWeight = false; 로 설정해야 합니다.
                % 하지만 보통 SAC에서는 알파를 자동 튜닝하는 것이 일반적입니다.
            end
            
            % (선택적) 초기 엔트로피 가중치 설정
            if isfield(rlHyperParams, 'initialEntropyWeight')
                entropyOpts.InitialEntropyWeight = rlHyperParams.initialEntropyWeight;
            end

            % 3. 설정된 entropyOpts 객체를 agentOpts에 할당
            agentOpts.EntropyWeightOptions = entropyOpts;
            
            % 액터 및 크리틱 옵션 (학습률 등)
            agentOpts.ActorOptimizerOptions = rlOptimizerOptions('LearnRate', rlHyperParams.actorLR, 'GradientThreshold', 1);
            agentOpts.CriticOptimizerOptions = rlOptimizerOptions('LearnRate', rlHyperParams.criticLR, 'GradientThreshold', 1);
            
            % obj.SACAgent = rlSACAgent(obj.ActorNetwork, {obj.CriticNetwork1, obj.CriticNetwork2}, agentOpts, ...
            %     'ActorOptimizerOptions', actorOptimizerOptions, ...
            %     'CriticOptimizerOptions', criticOptimizerOptions);
            disp('ActorNetwork:'); disp(class(obj.ActorNetwork));
            disp('Critic1: '); disp(class(obj.CriticNetwork1))
            disp('Critic2: '); disp(class(obj.CriticNetwork2))
            obj.agent = rlSACAgent(obj.ActorNetwork, [obj.CriticNetwork1 obj.CriticNetwork2], agentOpts);
            
            % 4. 학습 옵션 설정
            obj.TrainingOptions = rlTrainingOptions(...
                'MaxEpisodes', rlHyperParams.maxEpisodes, ...
                'MaxStepsPerEpisode', rlHyperParams.maxStepsPerEpisode, ...
                'ScoreAveragingWindowLength', rlHyperParams.scoreAveragingWindowLength, ...
                'StopTrainingCriteria', rlHyperParams.stopTrainingCriteria, ...
                'StopTrainingValue', rlHyperParams.stopTrainingValue, ...
                'SaveAgentCriteria', rlHyperParams.saveAgentCriteria, ...
                'SaveAgentValue', rlHyperParams.saveAgentValue, ...
                'SaveAgentDirectory', rlHyperParams.saveAgentPath, ...
                'Verbose', rlHyperParams.verbose, ...
                'Plots', rlHyperParams.plots, ...
                'UseParallel', rlHyperParams.useParallel); % 병렬 학습 사용 여부
            
            obj.TrainedAgentPath = rlHyperParams.saveAgentPath;
            if ~exist(obj.TrainedAgentPath, 'dir')
               mkdir(obj.TrainedAgentPath);
            end

            disp('--- SACTrainingManager 객체 초기화 완료 ---');
        end

        % --- 액터 신경망 생성 (비공개 헬퍼 메서드) ---
        function actorNetwork = createActorNetwork(obj, obsInfo, actInfo, networkArch)
            % 입력:
            %   obsInfo: 관찰 공간 명세
            %   actInfo: 행동 공간 명세
            %   networkArch: 구조체 .commonLayers (공통 계층), .meanLayers (평균 경로), .stdLayers (표준편차 경로)
            %                예: networkArch.commonLayers = [fullyConnectedLayer(128) reluLayer()];
            %                    networkArch.meanLayers = [fullyConnectedLayer(actInfo.Dimension(1)) tanhLayer()]; % 평균 출력
            %                    networkArch.stdLayers = [fullyConnectedLayer(actInfo.Dimension(1)) softplusLayer()]; % 표준편차 출력 (양수 보장)
            
            disp('액터 신경망 생성 중...');
            
            % 공통 경로
            commonPathLayers = [
                featureInputLayer(obsInfo.Dimension(1), 'Normalization', 'none', 'Name', 'observation')
                networkArch.commonLayers % networkArch.commonLayers 자체가 이미 레이어 배열임
            ];
            % 행동 평균을 위한 경로
            meanPathLayers = networkArch.meanLayers; % networkArch.meanLayers 자체가 이미 레이어 배열임
            % 행동 표준편차를 위한 경로
            stdPathLayers = networkArch.stdLayers;   % networkArch.stdLayers 자체가 이미 레이어 배열임
            
            % --- 테스트 끝 ---
            % 신경망 그래프 구성
            lGraph = layerGraph(commonPathLayers);
            lGraph = addLayers(lGraph, meanPathLayers);
            lGraph = addLayers(lGraph, stdPathLayers);
            
            % 경로 연결
            % commonPath의 마지막 레이어 이름을 알아야 함. 예시에서는 'common_relu_2'라고 가정
            % 또는 networkArch.commonLayers의 마지막 레이어 이름을 사용
            lastCommonLayerName = commonPathLayers(end).Name; % commonPath의 마지막 레이어 이름
            if isa(commonPathLayers(end), 'nnet.cnn.layer.ReluLayer') && length(commonPath) > 1 && isa(commonPath(end-1), 'nnet.cnn.layer.FullyConnectedLayer')
                 lastCommonLayerName = commonPathLayers(end-1).Name; % FC 레이어 이름으로 연결 시도
            end


            lGraph = connectLayers(lGraph, lastCommonLayerName, meanPathLayers(1).Name);
            lGraph = connectLayers(lGraph, lastCommonLayerName, stdPathLayers(1).Name);
            
            actorNetwork = dlnetwork(lGraph);
            actorNetwork = initialize(actorNetwork); % 선택적: 가중치 초기화
            
            % MATLAB의 rlContinuousGaussianActor는 평균과 로그 표준편차를 출력하는 네트워크를 기대함.
            % 따라서 stdPath의 마지막 활성화 함수는 로그 표준편차를 직접 출력하거나, 
            % 표준편차 출력 후 로그를 취하는 형태로 구성해야 할 수 있음.
            % 또는, rlContinuousGaussianActor가 내부적으로 처리하도록 구성.
            % 여기서는 dlnetwork를 직접 사용하므로, 에이전트가 이 출력을 어떻게 해석할지 명확해야 함.
            % rlSACAgent는 액터 신경망이 행동 분포의 파라미터를 출력하도록 기대합니다.
            % 가우시안 분포의 경우, 평균과 (로그)표준편차.
            % 만약 rlContinuousGaussianActor 객체를 사용한다면, 그 객체가 이 dlnetwork를 입력으로 받음.
            % 여기서는 dlnetwork를 직접 SACAgent에 전달.
            disp('액터 신경망 생성 완료.');
        end

        % --- 크리틱 신경망 생성 (비공개 헬퍼 메서드) ---
        function criticNetwork = createCriticNetwork(obj, obsInfo, actInfo, networkArch)
            % 입력:
            %   obsInfo: 관찰 공간 명세
            %   actInfo: 행동 공간 명세
            %   networkArch: 구조체 .statePath (상태 경로), .actionPath (행동 경로), .commonPath (결합 후 공통 경로)
            %                예: networkArch.statePath = [featureInputLayer(...) ...];
            %                    networkArch.actionPath = [featureInputLayer(...) ...];
            %                    networkArch.commonPath = [concatenationLayer(...) ... fullyConnectedLayer(1)];
            
            disp('크리틱 신경망 생성 중...');
            
            % 상태 경로
            statePath = [
                featureInputLayer(obsInfo.Dimension(1), 'Normalization', 'none', 'Name', 'observation')
                networkArch.statePath % 예: fullyConnectedLayer(128) reluLayer()
            ];
            
            % 행동 경로
            actionPath = [
                featureInputLayer(actInfo.Dimension(1), 'Normalization', 'none', 'Name', 'action')
                networkArch.actionPath % 예: fullyConnectedLayer(128) reluLayer()
            ];
            
            % 두 경로를 결합한 후의 공통 경로
            % concatenationLayer의 이름을 알아야 함. 예시에서는 'concat'
            % statePath와 actionPath의 마지막 레이어 이름을 알아야 함.
            lastStateLayerName = statePath(end).Name;
            lastActionLayerName = actionPath(end).Name;
            
            commonPath = [
                concatenationLayer(1, 2, 'Name', 'concat') % 두 입력을 채널 차원으로 결합
                networkArch.commonPath % 예: fullyConnectedLayer(256) reluLayer() fullyConnectedLayer(1)
            ];
            
            l = layerGraph();
            l = addLayers(l, statePath);
            l = addLayers(l, actionPath);
            l = addLayers(l, commonPath);
            
            l = connectLayers(l, lastStateLayerName, 'concat/in1');
            l = connectLayers(l, lastActionLayerName, 'concat/in2');
            
            criticNetwork = dlnetwork(l);
            % criticNetwork = initialize(criticNetwork); % 선택적

            disp('크리틱 신경망 생성 완료.');
        end
        
        %% --- 에이전트 학습 실행 ---
        function [trainedAgent, trainingStats] = trainAgent(obj)
            disp('--- SAC 에이전트 학습 시작 ---');
            
            % 학습 실행
            disp(class(obj.agent))
            obj.TrainingStats = train(obj.agent, obj.RLEnvironment, obj.TrainingOptions);

            % obj.agent = train_agent;
            % obj.TrainingStats = trainingState;
            
            trainedAgent = obj.agent;
            trainingStats = obj.TrainingStats;
            
            % 학습된 에이전트 저장 (선택적, TrainingOptions에서 이미 저장될 수 있음)
            agentToSave = obj.agent; % 현재 학습된 에이전트
            save(fullfile(obj.TrainedAgentPath, 'final_trained_sac_agent.mat'), 'agentToSave');
            disp(['최종 학습된 에이전트가 저장되었습니다: ', fullfile(obj.TrainedAgentPath, 'final_trained_sac_agent.mat')]);
            
            disp('--- SAC 에이전트 학습 완료 ---');
        end
        
        % --- 학습된 에이전트 반환 ---
        function agent = getTrainedAgent(obj)
            if isempty(obj.agent) || ~isfield(obj.TrainingStats, 'EpisodeReward') % 학습이 실행되었는지 간단히 확인
                warning('에이전트가 아직 학습되지 않았거나 학습 정보가 없습니다.');
                agent = [];
                return;
            end
            agent = obj.agent;
        end
        
        % --- 학습 통계 반환 ---
        function stats = getTrainingStats(obj)
            if isempty(obj.TrainingStats)
                warning('학습 통계가 없습니다.');
                stats = [];
                return;
            end
            stats = obj.TrainingStats;
        end

        %% --- 학습된 에이전트로 시뮬레이션 실행 (테스트용) ---
        function simData = simulateTrainedAgent(obj, numEpisodes, agentToSimulate)
            if nargin < 3
                agentToSimulate = obj.agent;
            end
            if isempty(agentToSimulate)
                error('시뮬레이션할 에이전트가 없습니다. 먼저 학습을 진행하거나 에이전트를 로드하세요.');
            end

            disp(['--- 학습된 에이전트로 ', num2str(numEpisodes), ' 에피소드 시뮬레이션 시작 ---']);
            simOpts = rlSimulationOptions('MaxSteps', obj.RLHyperParams.maxStepsPerEpisode, 'NumSimulations', numEpisodes);
            % 시뮬레이션 시 환경의 시각화 옵션을 켜려면 DroneSimulator 파라미터에서 설정해야 함.
            % 또는, 환경 객체를 새로 만들어서 전달할 수도 있음.
            % obj.RLEnvironment.DroneSim.EnableVisualization = true; % 임시로 켤 수 있으나, 권장 방식은 아님
            
            experience = sim(agentToSimulate, obj.RLEnvironment, simOpts);
            simData = experience; % experience는 SimulationOutput 객체 또는 배열
            disp('--- 시뮬레이션 완료 ---');
        end
        
        % --- 학습된 에이전트로 단일 에피소드를 실행하고 경로 데이터를 반환 ---
        function trajectory_data = runSingleEpisodeWithAgent(obj, agentToRun, enableVisualizationDuringRun)
            
            
            if nargin < 3
                enableVisualizationDuringRun = false; % 기본적으로 시각화 끔
            end
            
            disp('단일 에피소드 평가 실행 시작...');
            
            % 평가 실행을 위해 환경의 시각화 설정 임시 변경
            originalVisSetting = obj.RLEnvironment.DroneSim.EnableVisualization;
            obj.RLEnvironment.DroneSim.EnableVisualization = enableVisualizationDuringRun;
            
            % 비디오 녹화가 켜져 있었다면 평가 중에는 끄도록 처리 (선택적)
            originalVideoSetting = obj.RLEnvironment.DroneSim.EnableVideoRecording;
            obj.RLEnvironment.DroneSim.EnableVideoRecording = false; 
    
    
            obs = reset(obj.RLEnvironment); % 환경 리셋 (DroneSim 로그도 리셋됨)
            
            isDone = false;
            currentStepInEpisode = 0;
            
            while ~isDone && currentStepInEpisode < obj.RLHyperParams.maxStepsPerEpisode
                action = getAction(agentToRun, {obs}); % 에이전트로부터 행동 얻기
                action = action{1}; % cell에서 추출
                
                [nextObs, ~, isDone, ~] = step(obj.RLEnvironment, action); % 환경 스텝 진행
                obs = nextObs;
                currentStepInEpisode = currentStepInEpisode + 1;
                
                if enableVisualizationDuringRun
                    drawnow limitrate; % 시각화 업데이트
                end
            end
            
            % DroneSimulator 내부의 로그에서 경로 데이터 가져오기
            simResults = obj.RLEnvironment.DroneSim.getResults();
            trajectory_data.Time = simResults.Time;
            trajectory_data.PositionNED = simResults.Position; % [N, E, D]
            trajectory_data.EulerAngles = simResults.EulerAngles; % [R, P, Y]
            trajectory_data.Vel = simResults.Velocity; %[vx, vy, vz]
            
            % 원래 시각화 설정 복원
            obj.RLEnvironment.DroneSim.EnableVisualization = originalVisSetting;
            obj.RLEnvironment.DroneSim.EnableVideoRecording = originalVideoSetting;
    
            disp('단일 에피소드 평가 실행 완료.');
        end

    end
end