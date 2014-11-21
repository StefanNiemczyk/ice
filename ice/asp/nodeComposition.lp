% information/3		:		(entity,scope,representation)
% stream/3		: 		(system,source,information)
% node/3		:		(system,nodeName,entity)
% output/6 		:		(system,node,scope,rep,acc_mode,acc_modifier)
%--------------------------------------------------------------------------------------
% paramters
#const maxHopCount=2.
#const maxStepCount=6.
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


%--------------------------------------------------------------------------------------
#program query(k).

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
MIN { connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
		STEP < maxStepCount, 
		not after(IRO,PROVIDER) } MAX :- 
	input(SYSTEM,IRO,SCOPE,REP,ENTITY2,MIN,MAX),
	node(k,SYSTEM,IRO,ENTITY,ENTITY2),
	ENTITY2 != none,
	ENTITY2 != any.

MIN { connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
		STEP < maxStepCount, 
		not after(IRO,PROVIDER) } MAX :- 
	input(SYSTEM,IRO,SCOPE,REP,any,MIN,MAX),
	node(k,SYSTEM,IRO,ENTITY,ENTITY2).

MIN { connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none),STEP) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none),STEP), 
		STEP < maxStepCount, 
		not after(IRO,PROVIDER) } MAX :- 
	input(SYSTEM,IRO,SCOPE,REP,none,MIN,MAX),
	node(k,SYSTEM,IRO,ENTITY,ENTITY2).

% connect to node 2
MIN { connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,information(ENTITY2,SCOPE,REP,ENTITY),STEP) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY2,SCOPE,REP,ENTITY),STEP), 
		STEP < maxStepCount, 
		not after(IRO,PROVIDER),
		IRO != PROVIDER } MAX :- 
	input2(SYSTEM,IRO,SCOPE,REP,ENTITY,MIN,MAX), 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2),
	ENTITY2 != none,
	ENTITY2 != any.

MIN { connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,information(ENTITY2,SCOPE,REP,ENTITY),STEP) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY2,SCOPE,REP,ENTITY),STEP),
		STEP < maxStepCount, 
		not after(IRO,PROVIDER),
		IRO != PROVIDER } MAX :- 
	input2(SYSTEM,IRO,SCOPE,REP,any,MIN,MAX), 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2).

MIN { connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,information(ENTITY2,SCOPE,REP,none),STEP) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY2,SCOPE,REP,none),STEP), 
		STEP < maxStepCount, 
		not after(IRO,PROVIDER),
		IRO != PROVIDER } MAX :- 
	input2(SYSTEM,IRO,SCOPE,REP,none,MIN,MAX), 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2).

% create output
stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),VALUE+1) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	VALUE = #max{STEP : connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,_,_,_,STEP)},
	ENTITY2 != none,
	ENTITY2 != any,
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2).

stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),VALUE+1) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	VALUE = #max{STEP : connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,_,_,_,STEP)},
	output(SYSTEM,IRO,SCOPE,REP,none).

stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),VALUE+1) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	VALUE = #max{STEP : connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,_,_,_,STEP)},
	output(SYSTEM,IRO,SCOPE,REP,any).


% computing QoS parameters
%fix
metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP,FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	ENTITY2 != none,
	ENTITY2 != any,
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,fix,FIX_VALUE,MOD_VALUE).
metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP,FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,none), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,fix,FIX_VALUE,MOD_VALUE).
metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP,FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,any), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,fix,FIX_VALUE,MOD_VALUE).

%min
metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP,RESULT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
	ENTITY2 != none,
	ENTITY2 != any,
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,min,FIX_VALUE,MOD_VALUE),
	VALUE = #min{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	COUNT = #count{STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		IRO != PROVIDER, 
		STEP2 < STEP},
	STEP < maxStepCount,
	COUNT > 0,
	MOD_VALUE != 0,
	RESULT = VALUE+FIX_VALUE+MOD_VALUE*COUNT.

metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP,RESULT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
	ENTITY2 != none,
	ENTITY2 != any,
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,min,FIX_VALUE,MOD_VALUE),
	VALUE = #min{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	STEP < maxStepCount,
	MOD_VALUE == 0,
	RESULT = VALUE+FIX_VALUE.
%%%%
metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP,RESULT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,none), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,min,FIX_VALUE,MOD_VALUE),
	VALUE = #min{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	COUNT = #count{STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		IRO != PROVIDER, 
		STEP2 < STEP},
	STEP < maxStepCount,
	COUNT > 0,
	MOD_VALUE != 0,
	RESULT = VALUE+FIX_VALUE+MOD_VALUE*COUNT.

metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP,RESULT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,none), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,min,FIX_VALUE,MOD_VALUE),
	VALUE = #min{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	STEP < maxStepCount,
	MOD_VALUE == 0,
	RESULT = VALUE+FIX_VALUE.
%%%%
metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP,RESULT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,any), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,min,FIX_VALUE,MOD_VALUE),
	VALUE = #min{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	COUNT = #count{STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		IRO != PROVIDER,
		STEP2 < STEP},
	STEP < maxStepCount,
	COUNT > 0,
	MOD_VALUE != 0,
	RESULT = VALUE+FIX_VALUE+MOD_VALUE*COUNT.

metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP,RESULT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,any), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,min,FIX_VALUE,MOD_VALUE),
	VALUE = #min{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	STEP < maxStepCount,
	MOD_VALUE == 0,
	RESULT = VALUE+FIX_VALUE.


%avg
metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP,RESULT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
	ENTITY2 != none,
	ENTITY2 != any,
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,avg,FIX_VALUE,MOD_VALUE),
	VALUE = #sum{D,STEP2,PROVIDER,SOURCE,SCOPE2,REP2,ENTITY_REL : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,information(ENTITY,SCOPE2,REP2,ENTITY_REL),STEP2), 
		metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE2,REP2,ENTITY_REL),STEP2,D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	COUNT = #count{STEP2,PROVIDER,SOURCE,SCOPE2,REP2,ENTITY_REL : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,information(ENTITY,SCOPE2,REP2,ENTITY_REL),STEP2), 
		IRO != PROVIDER, 
		STEP2 < STEP},
	STEP < maxStepCount,
	COUNT > 0,
	RESULT = VALUE/COUNT+FIX_VALUE+MOD_VALUE*COUNT.

metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP,RESULT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,none), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,avg,FIX_VALUE,MOD_VALUE),
	VALUE = #sum{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	COUNT = #count{STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		IRO != PROVIDER, 
		STEP2 < STEP},
	STEP < maxStepCount,
	COUNT > 0,
	RESULT = VALUE/COUNT+FIX_VALUE+MOD_VALUE*COUNT.

metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP,RESULT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,any), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,avg,FIX_VALUE,MOD_VALUE),
	VALUE = #sum{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	COUNT = #count{STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		IRO != PROVIDER, 
		STEP2 < STEP},
	STEP < maxStepCount,
	COUNT > 0,
	RESULT = VALUE/COUNT+FIX_VALUE+MOD_VALUE*COUNT.

%metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),VALUE/COUNT+FIX_VALUE+MOD_VALUE*COUNT) :- 
%	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
%	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
%	metadataNode(METADATA,SYSTEM,IRO,avg,FIX_VALUE,MOD_VALUE),
%	VALUE = #sum{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
%	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
%	COUNT!=0.
%metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),VALUE/COUNT+FIX_VALUE+MOD_VALUE*COUNT) :- 
%	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
%	output(SYSTEM,IRO,SCOPE,REP,none), 
%	metadataNode(METADATA,SYSTEM,IRO,avg,FIX_VALUE,MOD_VALUE),
%	VALUE = #sum{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
%	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
%	COUNT!=0.
%metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),VALUE/COUNT+FIX_VALUE+MOD_VALUE*COUNT) :- 
%	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
%	output(SYSTEM,IRO,SCOPE,REP,any), 
%	metadataNode(METADATA,SYSTEM,IRO,avg,FIX_VALUE,MOD_VALUE),
%	VALUE = #sum{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
%	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
%	COUNT!=0.

%max
metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP,RESULT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
	ENTITY2 != none,
	ENTITY2 != any,
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	COUNT = #count{STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		IRO != PROVIDER, 
		STEP2 < STEP},
	STEP < maxStepCount,
	COUNT > 0,
	MOD_VALUE != 0,
	RESULT = VALUE+FIX_VALUE+MOD_VALUE*COUNT.

metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP,RESULT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
	ENTITY2 != none,
	ENTITY2 != any,
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	STEP < maxStepCount,
	MOD_VALUE == 0,
	RESULT = VALUE+FIX_VALUE.

metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP,RESULT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,none), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	COUNT = #count{STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		IRO != PROVIDER, 
		STEP2 < STEP},
	STEP < maxStepCount,
	COUNT > 0,
	MOD_VALUE != 0,
	RESULT = VALUE+FIX_VALUE+MOD_VALUE*COUNT.

metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP,RESULT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,none), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	STEP < maxStepCount,
	MOD_VALUE == 0,
	RESULT = VALUE+FIX_VALUE.

metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP,RESULT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,any), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	COUNT = #count{STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		IRO != PROVIDER,
		STEP2 < STEP},
	STEP < maxStepCount,
	COUNT > 0,
	MOD_VALUE != 0,
	RESULT = VALUE+FIX_VALUE+MOD_VALUE*COUNT.

metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP,RESULT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,any), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	STEP < maxStepCount,
	MOD_VALUE == 0,
	RESULT = VALUE+FIX_VALUE.


%metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
%	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
%	output(SYSTEM,IRO,SCOPE,REP,ENTITY2),
%	metadataNode(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
%	VALUE = #max{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)},
%	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
%	COUNT!=0.
%metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
%	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
%	output(SYSTEM,IRO,SCOPE,REP,none),
%	metadataNode(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
%	VALUE = #max{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)},
%	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
%	COUNT!=0.
%metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
%	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
%	output(SYSTEM,IRO,SCOPE,REP,any),
%	metadataNode(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
%	VALUE = #max{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)},
%	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
%	COUNT!=0.


%no input
metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP,FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,TYPE,FIX_VALUE,MOD_VALUE),
	COUNT = #count{STEP2,PROVIDER,SOURCE : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		IRO != PROVIDER,
		STEP2 < STEP},
	ENTITY2 != node,
	ENTITY2 != any,
	TYPE != fix,
	STEP < maxStepCount,
	COUNT == 0.
metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP,FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,none), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,TYPE,FIX_VALUE,MOD_VALUE),
	COUNT = #count{STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		IRO != PROVIDER,
		STEP2 < STEP},
	TYPE != fix,
	STEP < maxStepCount,
	COUNT == 0.
metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP,FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,any), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataNode(METADATA,SYSTEM,IRO,TYPE,FIX_VALUE,MOD_VALUE),
	COUNT = #count{STEP2,PROVIDER,SOURCE : 
		connectToNode(k,SYSTEM,IRO,ENTITY,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
		IRO != PROVIDER,
		STEP2 < STEP},
	TYPE != fix,
	STEP < maxStepCount,
	COUNT == 0.

%metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),FIX_VALUE) :- 
%	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
%	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
%	metadataNode(METADATA,SYSTEM,IRO,_,FIX_VALUE,MOD_VALUE),
%	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
%	COUNT==0.
%metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),FIX_VALUE) :- 
%	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
%	output(SYSTEM,IRO,SCOPE,REP,none), 
%	metadataNode(METADATA,SYSTEM,IRO,_,FIX_VALUE,MOD_VALUE),
%	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
%	COUNT==0.
%metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),FIX_VALUE) :- 
%	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
%	output(SYSTEM,IRO,SCOPE,REP,any), 
%	metadataNode(METADATA,SYSTEM,IRO,_,FIX_VALUE,MOD_VALUE),
%	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
%	COUNT==0.

:- metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP,VALUE), 
   metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP,VALUE2),
   VALUE != VALUE2.



%% transfer a stream between systems
0 { streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,STEP,TRANSFER_DELAY,TRANSFER_COST) } 1 :- 
	transfer(SYSTEM_SOURCE,SYSTEM_TARGET,TRANSFER_DELAY,TRANSFER_COST),
	stream(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,STEP), 
	STEP < maxStepCount,
	SYSTEM_TARGET != SOURCE.

:- stream(k,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,STEP2), stream(k,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,STEP), STEP != STEP2.

stream(k,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,STEP+1) :- 
	streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,STEP,TRANSFER_DELAY,TRANSFER_COST).


% computing QoS parameters	
metadataStream(k,METADATA,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,STEP+1,VALUE) :- 
	streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,STEP,TRANSFER_DELAY,TRANSFER_COST), 
	metadataStream(k,METADATA,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,STEP,VALUE), 
	STEP < maxStepCount,
	METADATA != delay.	

% hack to bound the delay computation. max 5 duplicated computations
metadataStream(k,delay,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,STEP+1,TRANSFER_DELAY+DELAY) :-
	streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,STEP,TRANSFER_DELAY,_), 
	metadataStream(k,delay,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,STEP,DELAY), 
	STEP < maxStepCount.



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
MIN { connectToMap(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
		entity(ENTITY,ENTITY_TYPE), 
		STEP < maxStepCount,
		not after(NODE,PROVIDER),
		NODE != PROVIDER } MAX :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	input(SYSTEM,NODE,SCOPE,REP,ENTITY2,MIN,MAX),
	ENTITY2 != none,
	ENTITY2 != any.

MIN { connectToMap(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none),STEP) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none),STEP), 
		entity(ENTITY,ENTITY_TYPE), 
		STEP < maxStepCount,
		not after(NODE,PROVIDER),
		NODE != PROVIDER } MAX :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	input(SYSTEM,NODE,SCOPE,REP,none,MIN,MAX).

MIN { connectToMap(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
		entity(ENTITY,ENTITY_TYPE), 
		STEP < maxStepCount,
		not after(NODE,PROVIDER),
		NODE != PROVIDER } MAX :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	input(SYSTEM,NODE,SCOPE,REP,any,MIN,MAX).

% outputs
map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,VALUE+1) :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	outputMap(SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2),
	VALUE = #max{STEP : connectToMap(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2,_,_,information(ENTITY,_,_,_),STEP), entity(ENTITY,ENTITY_TYPE)},
	ENTITY2 != any.


% computing QoS parameters
% fix
metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP,FIX_VALUE) :- 
	map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP),
	metadataMap(METADATA,SYSTEM,MAP,fix,FIX_VALUE,MOD_VALUE).

% min
metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP,RESULT) :- 
	map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP), 
	metadataMap(METADATA,SYSTEM,MAP,min,FIX_VALUE,MOD_VALUE),
	VALUE = #min{D,STEP2,PROVIDER,SOURCE,INFO : 
                connectToMap(k,SYSTEM,MAP,ENTITY_TYPE,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
                metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D),
                STEP2 < STEP},
	COUNT = #count{D,STEP2,PROVIDER,SOURCE,INFO : 
                connectToMap(k,SYSTEM,MAP,ENTITY_TYPE,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D),
                STEP2 < STEP},
	COUNT!=0,
	RESULT = FIX_VALUE+VALUE+COUNT*MOD_VALUE.

%avg
metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP,RESULT) :- 
	map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP), 
	metadataMap(METADATA,SYSTEM,MAP,avg,FIX_VALUE,MOD_VALUE),
	VALUE = #sum{D,STEP2,PROVIDER,SOURCE,INFO : 
                connectToMap(k,SYSTEM,MAP,ENTITY_TYPE,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
                metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D),
                STEP2 < STEP},
	COUNT = #count{D,STEP2,PROVIDER,SOURCE,INFO : 
                connectToMap(k,SYSTEM,MAP,ENTITY_TYPE,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D),
                STEP2 < STEP},
	COUNT!=0,
	RESULT = VALUE/COUNT+FIX_VALUE+COUNT*MOD_VALUE.

%max
metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP,RESULT) :- 
	map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP), 
	metadataMap(METADATA,SYSTEM,MAP,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,STEP2,PROVIDER,SOURCE,INFO : 
                connectToMap(k,SYSTEM,MAP,ENTITY_TYPE,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), 
                metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D),
                STEP2 < STEP},
	COUNT = #count{D,STEP2,PROVIDER,SOURCE,INFO : 
                connectToMap(k,SYSTEM,MAP,ENTITY_TYPE,ENTITY2,PROVIDER,SOURCE,INFO,STEP2), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D),
                STEP2 < STEP},
	COUNT!=0,
	RESULT = FIX_VALUE+VALUE+COUNT*MOD_VALUE.

%no input
metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP,FIX_VALUE) :- 
	map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP), 
	metadataMap(METADATA,SYSTEM,MAP,TYPE,FIX_VALUE,MOD_VALUE), 
	COUNT = #count{D,STEP2,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO,STEP2), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,STEP2,D)}, 
	COUNT == 0,
        TYPE != fix.



%%%
%%% Checking
%%% soft optimization contraint instead of a hard constraint to enable partial solutions
%%%

metadata(accuracy,maximize).
metadata(delay,minimize).

%% streams
0 { selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP) : stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP) } 1 :- requiredStream(SYSTEM,INFO), query(k).
:~ requiredStream(SYSTEM,INFO), not selectedStream(k,SYSTEM,_,_,INFO,_), query(k). [1@5,requiredStream]

% checking QoS parameters
:~ requiredStream(SYSTEM,INFO), 
   selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO,REQUIRED_VALUE), 
   metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,VALUE), 
   metadata(METADATA,maximize),
   REQUIRED_VALUE < VALUE,
   query(k). [1@4,requiredStream]
:~ requiredStream(SYSTEM,INFO), 
   selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO,REQUIRED_VALUE), 
   metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,VALUE),
   metadata(METADATA,minimize), 
   REQUIRED_VALUE > VALUE,
   query(k). [1@4,requiredStream]


%% maps
0 { selectedMap(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,STEP) : map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,STEP) } 1 :- 
	requiredMap(SYSTEM,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2), query(k).
:~ requiredMap(SYSTEM,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2), not selectedMap(k,SYSTEM,_,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,_), query(k). [1@5,requiredMap]

% checking QoS parameters
:~ selectedMap(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,STEP),
   requiredMetadata(METADATA,SYSTEM,ENTITY_TYPE,SCOPE,REP,ENTITY2,REQUIRED_VALUE), 
   metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP,VALUE), 
   metadata(METADATA,maximize),
   REQUIRED_VALUE < VALUE,
   query(k). [1@4,requiredStream]
:~ selectedMap(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,STEP),
   requiredMetadata(METADATA,SYSTEM,ENTITY_TYPE,SCOPE,REP,ENTITY2,REQUIRED_VALUE), 
   metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP,VALUE), 
   metadata(METADATA,minimize), 
   REQUIRED_VALUE > VALUE,
   query(k). [1@4,requiredStream]


% checking point budget of systems
:- system(SYSTEM,POINTS), SUM_STREAMS = #sum{COST,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,STEP : streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,STEP,_,COST)}, 
		SUM_NODES = #sum{COST,SYSTEM,NODE,ENTITY : node(k,SYSTEM,NODE,ENTITY), nodeCost(SYSTEM,NODE,COST)}, (SUM_NODES + SUM_STREAMS) > POINTS, query(k).



%%%
%%% Optimization
%%% QoS parameters of selected streams and maps are optimized
%%%

% stream QoS
#minimize { VALUE@priorityDelay,SYSTEM,PROVIDER,SOURCE,INFO,STEP : selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP), metadataStream(k,delay,SYSTEM,PROVIDER,SOURCE,INFO,STEP,VALUE), query(k) }.
#maximize { VALUE@priorityAccuracy,SYSTEM,PROVIDER,SOURCE,INFO,STEP : selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP), metadataStream(k,accuracy,SYSTEM,PROVIDER,SOURCE,INFO,STEP,VALUE), query(k) }.

% map QoS
#minimize { VALUE@priorityDelay,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP : selectedMap(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,STEP), 
		metadataMap(k,delay,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP,VALUE), query(k) }.
#maximize { VALUE@priorityAccuracy,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP : selectedMap(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,STEP), 
		metadataMap(k,accuracy,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP,VALUE), query(k) }.

% transfer cost
#minimize { TRANSFER_COST@priorityTransferCost,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,STEP : 
		streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,STEP,_,TRANSFER_COST), query(k) }.

% cost
sumCost(k,SUM_NODES + SUM_TRANSLATIONS + SUM_EXTRACTIONS) :- 
	SUM_NODES = #sum{COST,SYSTEM,NODE,ENTITY,ENTITY2 : node(k,SYSTEM,NODE,ENTITY,ENTITY2), nodeCost(SYSTEM,NODE,COST)}, 
	SUM_TRANSLATIONS = #sum{COST,SYSTEM,IRO,ENTITY,SCOPE : informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE), iroCost(SYSTEM,IRO,COST)},
	SUM_EXTRACTIONS = #count{SYSTEM,PROVIDER,SOURCE,SCOPE,REP1,INFO :  informationExtrection(k,SYSTEM,PROVIDER,SOURCE,SCOPE,REP1,INFO)}, 
	query(k).
#minimize { COST@priorityCost : sumCost(k,COST), query(k) }.


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









