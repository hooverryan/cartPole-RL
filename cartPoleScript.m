%% CartPole Simulation with DDPG
% This script goes through the training of a DDPG agent that controls the classical inverted pendulum.
% This first  agent will have only one action, the force on the cart.

%% Define Environment
clearvars
rng shuffle

qubeInit;

mdl='rlCartPole';
open_system(mdl)
VSS_CONTROL=0; % Basic Control

obsInfo = rlNumericSpec([4 1]);
obsInfo.Name = 'observation';
actInfo = rlNumericSpec(1);
actInfo.LowerLimit = minLimit;
actInfo.UpperLimit = maxLimit;
actInfo.Name = 'force';
env = rlSimulinkEnv('rlCartPole','rlCartPole/Controller/RL Agent',obsInfo,actInfo);
obsInfo = getObservationInfo(env);
numObservations = obsInfo.Dimension(1);
actInfo = getActionInfo(env);
numActions = actInfo.Dimension(1);

%% Create the critic network, actor network, and agent 
% Critic
statePath = [
    imageInputLayer([numObservations 1 1],'Normalization','none','Name','observation')
    fullyConnectedLayer(128,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(200,'Name','CriticStateFC2')];

actionPath = [
    imageInputLayer([numActions 1 1],'Normalization','none','Name','action')
    fullyConnectedLayer(200,'Name','CriticActionFC1','BiasLearnRateFactor',0)];

commonPath = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu')
    fullyConnectedLayer(1,'Name','CriticOutput')];

criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);
    
criticNetwork = connectLayers(criticNetwork,'CriticStateFC2','add/in1');
criticNetwork = connectLayers(criticNetwork,'CriticActionFC1','add/in2');

criticOptions = rlRepresentationOptions('LearnRate',1e-3,'useDevice','cpu');
critic = rlRepresentation(criticNetwork,obsInfo,actInfo,'Observation',{'observation'},'Action',{'action'},criticOptions);

% Actor
actorNetwork = [
    imageInputLayer([numObservations 1 1],'Normalization','none','Name','observation')
    fullyConnectedLayer(128,'Name','ActorFC1')
    reluLayer('Name','ActorRelu1')
    fullyConnectedLayer(200,'Name','ActorFC2')
    reluLayer('Name','ActorRelu2')
    fullyConnectedLayer(numActions,'Name','ActorFC3')
    tanhLayer('Name','ActorTanh1')
    scalingLayer('Name','ActorScaling','Scale',actInfo.UpperLimit)];

actorOptions = rlRepresentationOptions('LearnRate',5e-04,'useDevice','cpu');

actor = rlRepresentation(actorNetwork,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'ActorScaling'},actorOptions);

% Agent
agentOptions = rlDDPGAgentOptions(...
    'SampleTime',Ts,...
    'TargetSmoothFactor',1e-3,...
    'ExperienceBufferLength',1e6,...
    'MiniBatchSize',128);
agentOptions.NoiseOptions.Variance = 0.4;
agentOptions.NoiseOptions.VarianceDecayRate = 1e-5;
agent = rlDDPGAgent(actor,critic,agentOptions);

%% Training
maxepisodes = 2000;
maxsteps = ceil(Tf/Ts);
trainingOptions = rlTrainingOptions(...
    'MaxEpisodes',maxepisodes,...
    'MaxStepsPerEpisode',maxsteps,...
    'ScoreAveragingWindowLength',10,...
    'Verbose',false,...
    'Plots','training-progress',...
    'StopTrainingCriteria','AverageReward',...
    'StopTrainingValue',-400,...
    'SaveAgentDirectory','DDPG-basic',...
    'SaveAgentCriteria','AverageReward',...
    'SaveAgentValue',-400);

% Training and simulation can take hours.  Set trainAgent to false in order to use a pretrained agent.
trainAgent = false;
if trainAgent
  trainingStats = train(agent,env,trainingOptions);
else
  load('basicPreTrained.mat','agent');
end

sim(env,agent)

save('basic.mat','trainingStats','agent');
bdclose(mdl)

%% State Feedback DDPG Agent
% This agent will have four actions to produce the force on the cart.  This will mimic an LQR controller between the agent and the force.

%% Define Environment
clearvars
rng shuffle

qubeInit;

mdl='rlCartPole';
open_system(mdl)
VSS_CONTROL=1; % State Feeback Control

obsInfo = rlNumericSpec([4 1]);
obsInfo.Name = 'observation';
actInfo = rlNumericSpec([4 1]);
actInfo.LowerLimit = -300;
actInfo.UpperLimit = 300;
actInfo.Name = 'State Feeback Gains';
env = rlSimulinkEnv('rlCartPole','rlCartPole/Controller/RL Agent',obsInfo,actInfo);
obsInfo = getObservationInfo(env);
numObservations = obsInfo.Dimension(1);
actInfo = getActionInfo(env);
numActions = actInfo.Dimension(1);

%% Create the critic network, actor network, and agent 
% Critic
statePath = [
    imageInputLayer([numObservations 1 1],'Normalization','none','Name','observation')
    fullyConnectedLayer(128,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(200,'Name','CriticStateFC2')];

actionPath = [
    imageInputLayer([numActions 1 1],'Normalization','none','Name','action')
    fullyConnectedLayer(200,'Name','CriticActionFC1','BiasLearnRateFactor',0)];

commonPath = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu')
    fullyConnectedLayer(1,'Name','CriticOutput')];

criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);
    
criticNetwork = connectLayers(criticNetwork,'CriticStateFC2','add/in1');
criticNetwork = connectLayers(criticNetwork,'CriticActionFC1','add/in2');

criticOptions = rlRepresentationOptions('LearnRate',1e-3,'useDevice','cpu');
critic = rlRepresentation(criticNetwork,obsInfo,actInfo,'Observation',{'observation'},'Action',{'action'},criticOptions);

% Actor
actorNetwork = [
    imageInputLayer([numObservations 1 1],'Normalization','none','Name','observation')
    fullyConnectedLayer(128,'Name','ActorFC1')
    reluLayer('Name','ActorRelu1')
    fullyConnectedLayer(200,'Name','ActorFC2')
    reluLayer('Name','ActorRelu2')
    fullyConnectedLayer(numActions,'Name','ActorFC3')
    tanhLayer('Name','ActorTanh1')
    scalingLayer('Name','ActorScaling','Scale',actInfo.UpperLimit)];

actorOptions = rlRepresentationOptions('LearnRate',5e-04,'useDevice','cpu');

actor = rlRepresentation(actorNetwork,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'ActorScaling'},actorOptions);

% Agent
agentOptions = rlDDPGAgentOptions(...
    'SampleTime',Ts,...
    'TargetSmoothFactor',1e-3,...
    'ExperienceBufferLength',1e6,...
    'MiniBatchSize',128);
agentOptions.NoiseOptions.Variance = 0.4;
agentOptions.NoiseOptions.VarianceDecayRate = 1e-5;
agent = rlDDPGAgent(actor,critic,agentOptions);

%% Training
maxepisodes = 2000;
maxsteps = ceil(Tf/Ts);
trainingOptions = rlTrainingOptions(...
    'MaxEpisodes',maxepisodes,...
    'MaxStepsPerEpisode',maxsteps,...
    'ScoreAveragingWindowLength',10,...
    'Verbose',false,...
    'Plots','training-progress',...
    'StopTrainingCriteria','AverageReward',...
    'StopTrainingValue',-400,...
    'SaveAgentDirectory','DDPG-basic',...
    'SaveAgentCriteria','AverageReward',...
    'SaveAgentValue',-400);

trainingStats = train(agent,env,trainingOptions);

% Training and simulation can take hours.  Set trainAgent to false in order to use a pretrained agent.
trainAgent = false;
if trainAgent
  trainingStats = train(agent,env,trainingOptions);
else
  load('statefeedbackPreTrained.mat','agent');
end


save('statefeedback.mat','trainingStats','agent');
bdclose(mdl)
