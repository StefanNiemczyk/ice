% information/3		:		(entity,scope,representation)
% stream/3		: 		(system,source,information)
% node/3		:		(system,nodeName,entity)
% output/6 		:		(system,node,scope,rep,acc_mode,acc_modifier)
%--------------------------------------------------------------------------------------
% paramters
%#const maxHopCount=3.
%#const maxStepCount=10.
#const priorityDelay=3.
#const priorityAccuracy=3.
#const priorityTransferCost=2.
#const priorityCost=1.
 

%--------------------------------------------------------------------------------------
#program base.
%#show node/4.
%#show stream/5.
%#show connectToNode/6.
%#show streamTransfer/7.



metadata(accuracy,maximize).
metadata(delay,minimize).


%--------------------------------------------------------------------------------------
#program query(k,maxHopCount,maxStepCount).

#external query(k).

%%%
%%% Guessing
%%% Creating a system model of information processing
%%%

% cycle detection
after(NODE1,NODE2) :- connectToNode(k,SYSTEM,NODE2,_,_,NODE1,SOURCE,_,_).
after(NODE1,NODE2) :- connectToMap(k,SYSTEM,NODE2,ENTITY_TYPE,ENTITY2,NODE1,SOURCE,_,STEP).
after(NODE1,NODE3) :- after(NODE1,NODE2), after(NODE2,NODE3).
:- after(NODE,NODE).



%% creating a node
0 { node(k,SYSTEM,NODE,ENTITY,none) } 1 :- 
	sourceNode(SYSTEM,NODE,ENTITY), 
	entity(ENTITY,_), 
	query(k).
0 { node(k,SYSTEM,NODE,ENTITY,none) } 1 :- 
	nodeTemplate(SYSTEM,NODE,ENTITY_TYPE), 
	entity(ENTITY,ENTITY_TYPE), 
	query(k).
0 { node(k,SYSTEM,NODE,ENTITY,none) } 1 :- 
	nodeTemplate(SYSTEM,NODE,any), 
	entity(ENTITY,ENTITY_TYPE), 
	hasScope(ENTITY_TYPE,SCOPE), 
	input(SYSTEM,NODE,SCOPE,_,_,_,_), 
	query(k).

0 { node(k,SYSTEM,IRO,ENTITY,ENTITY2) } 1 :- 
	iro(SYSTEM,IRO,any,any), 
	entity(ENTITY,ENTITY_TYPE), 
	hasScope(ENTITY_TYPE,SCOPE), 
	input(SYSTEM,IRO,SCOPE,_,_,_,_),
	entity(ENTITY2,ENTITY_TYPE2), 
	hasScope(ENTITY_TYPE2,SCOPE2), 
	input2(SYSTEM,IRO,SCOPE2,_,_,_,_),
	query(k).
0 { node(k,SYSTEM,IRO,ENTITY,none) } 1 :- 
	iro(SYSTEM,IRO,any,none), 
	entity(ENTITY,ENTITY_TYPE), 
	hasScope(ENTITY_TYPE,SCOPE), 
	input(SYSTEM,IRO,SCOPE,_,_,_,_),
	query(k).

% connect to node
MIN { connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP)) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
		STEP < maxStepCount, 
		not after(IRO,PROVIDER) } MAX :- 
	input(SYSTEM,IRO,SCOPE,REP,ENTITY2,MIN,MAX),
	node(k,SYSTEM,IRO,ENTITY,ENTITY2),
	ENTITY2 != none,
	ENTITY2 != any.

MIN { connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP)) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
		STEP < maxStepCount, 
		not after(IRO,PROVIDER) } MAX :- 
	input(SYSTEM,IRO,SCOPE,REP,any,MIN,MAX),
	node(k,SYSTEM,IRO,ENTITY,ENTITY2).

MIN { connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none),STEP)) :
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none),STEP), 
		STEP < maxStepCount, 
		not after(IRO,PROVIDER) } MAX :- 
	input(SYSTEM,IRO,SCOPE,REP,none,MIN,MAX),
	node(k,SYSTEM,IRO,ENTITY,ENTITY2).


% connect to node 2
MIN { connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY2,SCOPE,REP,ENTITY),STEP)) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY2,SCOPE,REP,ENTITY),STEP), 
		STEP < maxStepCount, 
		not after(IRO,PROVIDER),
		IRO != PROVIDER } MAX :- 
	input2(SYSTEM,IRO,SCOPE,REP,ENTITY,MIN,MAX), 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2),
	ENTITY2 != none,
	ENTITY2 != any.

MIN { connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY2,SCOPE,REP,ENTITY),STEP)) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY2,SCOPE,REP,ENTITY),STEP),
		STEP < maxStepCount, 
		not after(IRO,PROVIDER),
		IRO != PROVIDER } MAX :- 
	input2(SYSTEM,IRO,SCOPE,REP,any,MIN,MAX), 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2).

MIN { connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY2,SCOPE,REP,none),STEP)) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY2,SCOPE,REP,none),STEP), 
		STEP < maxStepCount, 
		not after(IRO,PROVIDER),
		IRO != PROVIDER } MAX :- 
	input2(SYSTEM,IRO,SCOPE,REP,none,MIN,MAX), 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2).


% create output
nodeCount(node(k,SYSTEM,IRO,ENTITY,ENTITY2), MINV+MINV2, MAXV+MAXV2) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	MINV = #sum{MIN,SCOPE,REP,ENT2 : input(SYSTEM,IRO,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV = #sum{MAX,SCOPE,REP,ENT2 : input(SYSTEM,IRO,SCOPE,REP,ENT2,_,MAX)}, 
	MINV2 = #sum{MIN,SCOPE,REP,ENT2 : input2(SYSTEM,IRO,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV2 = #sum{MAX,SCOPE,REP,ENT2 : input2(SYSTEM,IRO,SCOPE,REP,ENT2,_,MAX)}.

nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP_MAX, COUNT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2),
	STEP_MAX = #max{STEP : connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,_,_,_,_,STEP))},
	COUNT = #count{PROVIDER,SOURCE,INFO,STEP2 : connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)), 
		PROVIDER != IRO, 
		STEP2 < STEP_MAX+1},
	nodeCount(node(k,SYSTEM,IRO,ENTITY,ENTITY2), MIN, MAX),
	COUNT > MIN-1,
	COUNT < MAX+1,
	STEP_MAX != #inf.

nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), 0, 0) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2),
	STEP_MAX = #max{STEP : connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,_,_,_,_,STEP))},
	COUNT = #count{PROVIDER,SOURCE,INFO,STEP2 : connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)), 
		PROVIDER != IRO, 
		STEP2 < STEP_MAX+1},
	nodeCount(node(k,SYSTEM,IRO,ENTITY,ENTITY2), MIN, MAX),
	COUNT > MIN-1,
	COUNT < MAX+1,
	STEP_MAX == #inf.


stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP_MAX+1) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2),STEP_MAX,_),
	ENTITY2 != none,
	ENTITY2 != any,
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2).

stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP_MAX+1) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2),STEP_MAX,_),
	output(SYSTEM,IRO,SCOPE,REP,none).

stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP_MAX+1) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2),STEP_MAX,_),
	output(SYSTEM,IRO,SCOPE,REP,any).



% computing QoS parameters
%fix
metadataStream(k,stream(METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP),FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	ENTITY2 != none,
	ENTITY2 != any,
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,fix,FIX_VALUE,MOD_VALUE).
metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP),FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,none), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,fix,FIX_VALUE,MOD_VALUE).
metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP),FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,any), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,fix,FIX_VALUE,MOD_VALUE).

%min
metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
	ENTITY2 != none,
	ENTITY2 != any,
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,min,FIX_VALUE,MOD_VALUE),
	VALUE = #min{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)),
		metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2),D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, COUNT),
	COUNT > 0.

metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,none), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,min,FIX_VALUE,MOD_VALUE),
	VALUE = #min{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)),
		metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2),D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, COUNT),
	COUNT > 0.

metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,any), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,min,FIX_VALUE,MOD_VALUE),
	VALUE = #min{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)),
		metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2),D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, COUNT),
	COUNT > 0.


%avg
metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP),VALUE/COUNT+FIX_VALUE+MOD_VALUE*COUNT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
	ENTITY2 != none,
	ENTITY2 != any,
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,avg,FIX_VALUE,MOD_VALUE),
	VALUE = #sum{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)),
		metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2),D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, COUNT),
	COUNT > 0.

metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP),VALUE/COUNT+FIX_VALUE+MOD_VALUE*COUNT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,none), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,avg,FIX_VALUE,MOD_VALUE),
	VALUE = #sum{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)),
		metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2),D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, COUNT),
	COUNT > 0.

metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP),VALUE/COUNT+FIX_VALUE+MOD_VALUE*COUNT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,any), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,avg,FIX_VALUE,MOD_VALUE),
	VALUE = #sum{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)),
		metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2),D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, COUNT),
	COUNT > 0.


%max
metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
	ENTITY2 != none,
	ENTITY2 != any,
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)),
		metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2),D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, COUNT),
	COUNT > 0.

metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,none), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)),
		metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2),D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, COUNT),
	COUNT > 0.

metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,any), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)),
		metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2),D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, COUNT),
	COUNT > 0.


%no input
metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP),FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
	ENTITY2 != none,
	ENTITY2 != any,
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,TYPE,FIX_VALUE,MOD_VALUE),
	TYPE != fix,
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, 0).

metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP),FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,none), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,TYPE,FIX_VALUE,MOD_VALUE),
	TYPE != fix,
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, 0).

metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP),FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,any), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,TYPE,FIX_VALUE,MOD_VALUE),
	TYPE != fix,
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, 0).


%% Speed up things
:- metadataStream(k,METADATA,STREAM,VALUE), 
   metadataStream(k,METADATA,STREAM,VALUE2),
   VALUE != VALUE2.
%:- nodeMeta(NODE, STEP_MAX, COUNT), nodeMeta(NODE, STEP_MAX2, COUNT2), STEP_MAX != STEP_MAX2.
%:- nodeMeta(NODE, STEP_MAX, COUNT), nodeMeta(NODE, STEP_MAX2, COUNT2), COUNT != COUNT2.

:- stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2), 
   stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP), 
   STEP != STEP2.

%transfer(SYSTEM_1,SYSTEM_3,TRANSFER_DELAY_1+TRANSFER_DELAY_2,TRANSFER_COST_1+TRANSFER_COST_2) :- 
%	transfer(SYSTEM_1,SYSTEM_2,TRANSFER_DELAY_1,TRANSFER_COST_1),
%	transfer(SYSTEM_2,SYSTEM_3,TRANSFER_DELAY_2,TRANSFER_COST_2).

%requiredInformation(k,SYSTEM,INFO) :- requiredStream(SYSTEM,INFO).
%requiredInformation(k,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2)) :- node(k,SYSTEM,NODE,ENTITY,ENTITY2), input(SYSTEM,NODE,SCOPE,REP,ENTITY2,MIN,MAX).
%requiredInformation(k,SYSTEM,information(ENTITY,SCOPE,REP,none)) :- node(k,SYSTEM,NODE,ENTITY,ENTITY2), input(SYSTEM,NODE,SCOPE,REP,none,MIN,MAX).
%requiredInformation(k,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2)) :- node(k,SYSTEM,NODE,ENTITY,ENTITY2), input(SYSTEM,NODE,SCOPE,REP,any,MIN,MAX).
%requiredInformation(k,SYSTEM,information(ENTITY2,SCOPE,REP,ENTITY)) :- node(k,SYSTEM,NODE,ENTITY,ENTITY2), input2(SYSTEM,NODE,SCOPE,REP,ENTITY,MIN,MAX).
%requiredInformation(k,SYSTEM,information(ENTITY2,SCOPE,REP,none)) :- node(k,SYSTEM,NODE,ENTITY,ENTITY2), input(SYSTEM,NODE,SCOPE,REP,none,MIN,MAX).
%requiredInformation(k,SYSTEM,information(ENTITY2,SCOPE,REP,ENTITY)) :- node(k,SYSTEM,NODE,ENTITY,ENTITY2), input(SYSTEM,NODE,SCOPE,REP,any,MIN,MAX).
%requiredInformation(k,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2)) :- mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), entity(ENTITY,ENTITY_TYPE), input(SYSTEM,NODE,SCOPE,REP,_,MIN,MAX).


%:- streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE1,PROVIDER,SOURCE,INFO,_),_,_,_), streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE2,PROVIDER,SOURCE,INFO,_),_,_,_), SYSTEM_SOURCE1 != SYSTEM_SOURCE2.

%% transfer a stream between systems
%*
0 { streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP+1),TRANSFER_DELAY,TRANSFER_COST,HOPS+1) : 
		transfer(SYSTEM_SOURCE,SYSTEM_TARGET,TRANSFER_DELAY,TRANSFER_COST), 
		streamTransfer(k,SYSTEM_SOURCE,stream(k,SYSTEM_TRANSFER_SOURCE,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP),_,_,HOPS),
		HOPS < maxHopCount,
		STEP < maxStepCount } 1 :- 
	system(SYSTEM_TARGET),
	stream(k,_,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP),
	SYSTEM_TARGET != SOURCE.


0 { streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,STEP),TRANSFER_DELAY,TRANSFER_COST,HOPS+1) } 1 :- 
	transfer(SYSTEM_SOURCE,SYSTEM_TARGET,TRANSFER_DELAY,TRANSFER_COST),
	streamTransfer(k,SYSTEM_SOURCE,stream(k,SYSTEM_TRANSFER_SOURCE,PROVIDER,SOURCE,INFO,STEP-1),_,_,HOPS),
	SYSTEM_TRANSFER_SOURCE != SYSTEM_TARGET,
	%SYSTEM_TRANSFER_SOURCE != SYSTEM_SOURCE,
	%requiredInformation(k,SYSTEM_TARGET,INFO),
	%stream(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,STEP),
	not stream(k,SYSTEM_SOURCE2,PROVIDER,SOURCE,INFO,_),
	SYSTEM_SOURCE != SYSTEM_SOURCE2,
	system(SYSTEM_SOURCE2),
	HOPS < maxHopCount,
	STEP < maxStepCount,
	SYSTEM_TARGET != SOURCE.

0 { streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_SOURCE,INFO,STEP),TRANSFER_DELAY,TRANSFER_COST,1) } 1 :- 
	transfer(SYSTEM_SOURCE,SYSTEM_TARGET,TRANSFER_DELAY,TRANSFER_COST),
	requiredInformation(k,SYSTEM_TARGET,INFO),
	stream(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_SOURCE,INFO,STEP), 
	maxHopCount > 0,
	STEP < maxStepCount,
	SYSTEM_TARGET != SYSTEM_SOURCE.
*%
stream(k,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,STEP+1) :- 
	streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,STEP),TRANSFER_DELAY,TRANSFER_COST,_).


% computing QoS parameters	
metadataStream(k,METADATA,stream(k,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,STEP+1),VALUE) :- 
	streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,STEP),TRANSFER_DELAY,TRANSFER_COST,_), 
	metadataStream(k,METADATA,stream(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,STEP),VALUE), 
	METADATA != delay.	

% computing delay
metadataStream(k,delay,stream(k,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,STEP+1),TRANSFER_DELAY+DELAY) :-
	streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,STEP),TRANSFER_DELAY,TRANSFER_COST,_), 
	metadataStream(k,delay,stream(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,STEP),DELAY).



%% extract from representation
%0 { informationExtrection(k,SYSTEM,PROVIDER,SOURCE,SCOPE,REP1,information(ENTITY,SUB_SCOPE,REP2,none)) } 1 :- 
%	stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP1,none)), 
%	hasDimension(REP1,SUB_SCOPE,_,_), 
%	hasRepresentation(SUB_SCOPE,REP2), 
%	query(k).
%stream(k,SYSTEM,PROVIDER,SOURCE,INFO) :- 
%	informationExtrection(k,SYSTEM,PROVIDER,SOURCE,SCOPE,REP1,INFO).


% computing QoS parameters	
%metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SUB_SCOPE,REP2,ENTITY2),ACCURACY) :- 
%	informationExtrection(k,SYSTEM,PROVIDER,SOURCE,SCOPE,REP1,information(ENTITY,SUB_SCOPE,REP2,ENTITY2)),
%	metadataStream(k,METADATA,SYSTEM_SOURCE,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP1,ENTITY2),ACCURACY).



%% creating a map
0 { mapNode(k,SYSTEM,NODE,ENTITY_TYPE,none) } 1 :- 
	mapNodeTemplate(SYSTEM,NODE,ENTITY_TYPE),
	query(k).

% inputs
MIN { connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP)) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
		entity(ENTITY,ENTITY_TYPE), 
		STEP < maxStepCount,
		not after(NODE,PROVIDER),
		NODE != PROVIDER } MAX :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	input(SYSTEM,NODE,SCOPE,REP,ENTITY2,MIN,MAX),
	ENTITY2 != none,
	ENTITY2 != any.

MIN { connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none),STEP)) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none),STEP), 
		entity(ENTITY,ENTITY_TYPE), 
		STEP < maxStepCount,
		not after(NODE,PROVIDER),
		NODE != PROVIDER } MAX :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	input(SYSTEM,NODE,SCOPE,REP,none,MIN,MAX).

MIN { connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP)) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
		entity(ENTITY,ENTITY_TYPE), 
		STEP < maxStepCount,
		not after(NODE,PROVIDER),
		NODE != PROVIDER } MAX :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	input(SYSTEM,NODE,SCOPE,REP,any,MIN,MAX).

% outputs
mapCount(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), MINV, MAXV) :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	MINV = #sum{MIN,SCOPE,REP,ENT2 : input(SYSTEM,NODE,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV = #sum{MAX,SCOPE,REP,ENT2 : input(SYSTEM,NODE,SCOPE,REP,ENT2,_,MAX)}.

mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), STEP_MAX, COUNT) :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),
	STEP_MAX = #max{STEP : connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), stream(k,_,_,_,_,STEP))},
	COUNT = #count{PROVIDER,SOURCE,INFO,STEP2 : connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)), 
		PROVIDER != NODE, 
		STEP2 < STEP_MAX+1},
	mapCount(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), MIN, MAX),
	COUNT > MIN-1,
	COUNT < MAX+1,
	STEP_MAX != #inf.

mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 0, 0) :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),
	STEP_MAX = #max{STEP : connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), stream(k,_,_,_,_,STEP))},
	COUNT = #count{PROVIDER,SOURCE,INFO,STEP2 : connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)), 
		PROVIDER != NODE, 
		STEP2 < STEP_MAX+1},
	mapCount(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), MIN, MAX),
	COUNT > MIN-1,
	COUNT < MAX+1,
	STEP_MAX == #inf.


map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP_MAX+1) :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	outputMap(SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2),
	mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), STEP_MAX, COUNT),
	ENTITY2 != any.


% computing QoS parameters
% fix
metadataMap(k,METADATA,map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP),FIX_VALUE) :- 
	map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP),
	metadataMap(METADATA,SYSTEM,NODE,fix,FIX_VALUE,MOD_VALUE).

% min
metadataMap(k,METADATA,map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP),FIX_VALUE+VALUE+COUNT*MOD_VALUE) :- 
	map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP), 
	metadataMap(METADATA,SYSTEM,NODE,min,FIX_VALUE,MOD_VALUE),
	VALUE = #min{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)),
		metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2),D), 
		NODE != PROVIDER, 
		STEP2 < STEP}, 
	mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,COUNT),
	COUNT != 0.

%avg
metadataMap(k,METADATA,map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP),FIX_VALUE+VALUE/COUNT+COUNT*MOD_VALUE) :- 
	map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP), 
	metadataMap(METADATA,SYSTEM,NODE,avg,FIX_VALUE,MOD_VALUE),
	VALUE = #sum{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)),
		metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2),D), 
		NODE != PROVIDER, 
		STEP2 < STEP}, 
	mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,COUNT),
	COUNT != 0.

%max
metadataMap(k,METADATA,map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP),FIX_VALUE+VALUE+COUNT*MOD_VALUE) :- 
	map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP), 
	metadataMap(METADATA,SYSTEM,NODE,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)),
		metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2),D), 
		NODE != PROVIDER, 
		STEP2 < STEP}, 
	mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,COUNT),
	COUNT != 0.

%no input
metadataMap(k,METADATA,map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP),FIX_VALUE) :- 
	map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP), 
	metadataMap(METADATA,SYSTEM,NODE,TYPE,FIX_VALUE,MOD_VALUE),
	mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,0),
        TYPE != fix.



%%%
%%% Checking
%%% soft optimization contraint instead of a hard constraint to enable partial solutions
%%%

%% streams
0 { selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP) : stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP) } 1 :- requiredStream(SYSTEM,INFO), query(k).
:~ requiredStream(SYSTEM,INFO), not selectedStream(k,SYSTEM,_,_,INFO,_), query(k). [1@5,requiredStream]

%#minimize { 1@5,SYSTEM,INFO :  requiredStream(SYSTEM,INFO), not selectedStream(k,SYSTEM,_,_,INFO,_), query(k) }.
%??? why?

% checking QoS parameters

:~ requiredStream(SYSTEM,INFO), 
   selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO,REQUIRED_VALUE), 
   metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),VALUE), 
   metadata(METADATA,maximize),
   REQUIRED_VALUE < VALUE,
   query(k). [1@4,requiredStream]
:~ requiredStream(SYSTEM,INFO), 
   selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO,REQUIRED_VALUE), 
   metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),VALUE),
   metadata(METADATA,minimize), 
   REQUIRED_VALUE > VALUE,
   query(k). [1@4,requiredStream]


%% maps
0 { selectedMap(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,STEP) : map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,STEP) } 1 :- 
	requiredMap(SYSTEM,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2), query(k).
:~ requiredMap(SYSTEM,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2), not selectedMap(k,SYSTEM,_,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,_), query(k). [1@5,requiredMap]

% checking QoS parameters
:~ selectedMap(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,STEP),
   requiredMetadata(METADATA,SYSTEM,ENTITY_TYPE,SCOPE,REP,ENTITY2,REQUIRED_VALUE), 
   metadataMap(k,METADATA,map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP),VALUE),
   metadata(METADATA,maximize),
   REQUIRED_VALUE < VALUE,
   query(k). [1@4,requiredStream]
:~ selectedMap(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,STEP),
   requiredMetadata(METADATA,SYSTEM,ENTITY_TYPE,SCOPE,REP,ENTITY2,REQUIRED_VALUE), 
   metadataMap(k,METADATA,map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP),VALUE),
   metadata(METADATA,minimize), 
   REQUIRED_VALUE > VALUE,
   query(k). [1@4,requiredStream]


% checking point budget of systems
:- system(SYSTEM,POINTS), SUM_STREAMS = #sum{COST,SYSTEM,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,STEP : streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),_,COST,_)}, 
		SUM_NODES = #sum{COST,SYSTEM,NODE,ENTITY,ENTITY2 : node(k,SYSTEM,NODE,ENTITY,ENTITY2), nodeCost(SYSTEM,NODE,COST)}, (SUM_NODES + SUM_STREAMS) > POINTS, query(k).



%%%
%%% Optimization
%%% QoS parameters of selected streams and maps are optimized
%%%

%sumMetadataStream(k,METADATA,SUM) :- 
%	metadata(METADATA,_), 
%	SUM = #sum{VALUE,INFO,SYSTEM,PROVIDER,SOURCE,STEP : selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP), metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),VALUE)}, 
%	query(k).

%#minimize {sum@3,METADATA : metadata(METADATA,_), sumMetadataStream(k,METADATA,SUM), query(k) }.

% stream QoS
#minimize { VALUE@priorityDelay,INFO,SYSTEM,PROVIDER,SOURCE,STEP : selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP), metadataStream(k,delay,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),VALUE), query(k) }.
#maximize { VALUE@priorityAccuracy,INFO,SYSTEM,PROVIDER,SOURCE,STEP : selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP), metadataStream(k,accuracy,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),VALUE), query(k) }.

% map QoS
#minimize { VALUE@priorityDelay,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP : selectedMap(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP), 
		metadataMap(k,delay,map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP),VALUE), query(k) }.
#maximize { VALUE@priorityAccuracy,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP : selectedMap(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP), 
		metadataMap(k,accuracy,map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP),VALUE), query(k) }.

% transfer cost
#minimize { TRANSFER_COST@priorityTransferCost,SYSTEM,STREAM : streamTransfer(k,SYSTEM,STREAM,_,TRANSFER_COST,_), query(k) }.

% cost
sumCost(k,SUM_NODES) :- 
	SUM_NODES = #sum{COST,SYSTEM,NODE,ENTITY,ENTITY2 : node(k,SYSTEM,NODE,ENTITY,ENTITY2), nodeCost(SYSTEM,NODE,COST)}, 
	%SUM_TRANSLATIONS = #sum{COST,SYSTEM,IRO,ENTITY,SCOPE : informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE), iroCost(SYSTEM,IRO,COST)},
	%SUM_EXTRACTIONS = #count{SYSTEM,PROVIDER,SOURCE,SCOPE,REP1,INFO :  informationExtrection(k,SYSTEM,PROVIDER,SOURCE,SCOPE,REP1,INFO)}, 
	query(k).
#minimize { COST@priorityCost : sumCost(k,COST), query(k) }.
%#minimize { COST@priorityCost,SYSTEM,NODE,ENTITY,ENTITY2 : node(k,SYSTEM,NODE,ENTITY,ENTITY2), nodeCost(SYSTEM,NODE,COST), query(k) }.



%--------------------------------------------------------------------------------------
#program entity(entity,entityType).

entity(entity,entityType).


%--------------------------------------------------------------------------------------
#program sourceNode(node,system,source,entity,scope,rep,entity2,delayFix,accuracyFix,cost).

#external sourceNode(system,node,entity).

output(system,node,scope,rep,none).

metadataNode(accuracy,system,node,fix,accuracyFix,0).
metadataNode(delay,system,node,fix,delayFix,0).
nodeCost(system,node,cost).


%--------------------------------------------------------------------------------------
%#program map(system,map,scope,rep,entity2,minCount,maxCount,delayType,delayFix,delayMod,accuracyType,accuracyFix,accuracyMod).

%#external nodeTemplate(system,map,scope,rep,entity2). 
%inputMap(system,map,minCount,maxCount).
%metadataMap(delay,system,map,delayType,delayFix,delayMod).
%metadataMap(accuracy,system,map,accuracyType,accuracyFix,accuracyMod).


%--------------------------------------------------------------------------------------
#program requiredStream(system,information,delayValue,accuracyValue).

#external requiredStream(system,information).
requiredMetadata(accuracy,system,information,accuracyValue).
requiredMetadata(delay,system,information,delayValue).


%--------------------------------------------------------------------------------------
#program requiredMap(system,entity_type,scope,representation,entity2).

#external requiredMap(system,entity_type,scope,representation,entity2).


%--------------------------------------------------------------------------------------
#program system(name,points).

#external system(name,points).


%--------------------------------------------------------------------------------------
#program transfer(system1,system2,delay,cost).

#external transfer(system1,system2,delay,cost).
transfer(system2,system1,delay,cost) :- transfer(system1,system2,delay,cost).









