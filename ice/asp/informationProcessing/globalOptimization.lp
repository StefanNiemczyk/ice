%%%%--------------------------------------------------------------------------------------
%%%%
%%%% Global optimization
%%%%
%%%% Implements a global optimization and propagation of QoS metadata  
%%%%
%%%%--------------------------------------------------------------------------------------
#program query(k,maxHopCount,maxStepCount).

% computing stream QoS metadata 
%fix
metadataStream(k,stream(METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP),FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	ENTITY2 != none,
	ENTITY2 != any,
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataOutput(METADATA,SYSTEM,IRO,fix,FIX_VALUE,_).
metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP),FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,none), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP), 
	metadataOutput(METADATA,SYSTEM,IRO,fix,FIX_VALUE,_).
metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP),FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,any), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataOutput(METADATA,SYSTEM,IRO,fix,FIX_VALUE,_).

%min
metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
	ENTITY2 != none,
	ENTITY2 != any,
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataOutput(METADATA,SYSTEM,IRO,min,FIX_VALUE,MOD_VALUE),
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
	metadataOutput(METADATA,SYSTEM,IRO,min,FIX_VALUE,MOD_VALUE),
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
	metadataOutput(METADATA,SYSTEM,IRO,min,FIX_VALUE,MOD_VALUE),
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
	metadataOutput(METADATA,SYSTEM,IRO,avg,FIX_VALUE,MOD_VALUE),
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
	metadataOutput(METADATA,SYSTEM,IRO,avg,FIX_VALUE,MOD_VALUE),
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
	metadataOutput(METADATA,SYSTEM,IRO,avg,FIX_VALUE,MOD_VALUE),
	VALUE = #sum{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)),
		metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2),D), 
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, COUNT),
	COUNT > 0.


%max
inputMetaNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2),METADATA,VALUE) :-
	%metadataOutput(METADATA,SYSTEM,IRO,max,_,_),
	connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STREAM),
	metadataStream(k,METADATA,STREAM,VALUE).

metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
	ENTITY2 != none,
	ENTITY2 != any,
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataOutput(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D : inputMetaNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2),METADATA,D)}, 
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, COUNT),
	COUNT > 0.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,none), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP), 
	metadataOutput(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D : inputMetaNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2),METADATA,D)}, 
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, COUNT),
	COUNT > 0.

%*
metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,none), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP), 
	metadataOutput(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,STEP2,PROVIDER,SOURCE,INFO : 
		connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)),
		metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2),D),
		IRO != PROVIDER, 
		STEP2 < STEP}, 
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, COUNT),
	COUNT > 0.
*%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,any), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataOutput(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D : inputMetaNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2),METADATA,D)}, 
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, COUNT),
	COUNT > 0.


%no input
metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP),FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,ENTITY2), 
	ENTITY2 != none,
	ENTITY2 != any,
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataOutput(METADATA,SYSTEM,IRO,TYPE,FIX_VALUE,_),
	TYPE != fix,
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, 0).

metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP),FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,none), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,none),STEP), 
	metadataOutput(METADATA,SYSTEM,IRO,TYPE,FIX_VALUE,_),
	TYPE != fix,
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, 0).

metadataStream(k,METADATA,stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP),FIX_VALUE) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP,any), 
	stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataOutput(METADATA,SYSTEM,IRO,TYPE,FIX_VALUE,_),
	TYPE != fix,
	nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP-1, 0).


%% Speed up
:- metadataStream(k,METADATA,STREAM,VALUE), 
   metadataStream(k,METADATA,STREAM,VALUE2),
   VALUE != VALUE2.


%% Transfer of streams
% computing QoS metadata	
metadataStream(k,METADATA,stream(k,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,STEP+1),VALUE) :- 
	streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,STEP),_), 
	metadataStream(k,METADATA,stream(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,STEP),VALUE), 
	not metadataOutput(METADATA,SYSTEM_SOURCE,SYSTEM_TARGET,_).	

metadataStream(k,METADATA,stream(k,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,STEP+1),VALUE+VALUE_PROCESSING) :-
	streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,STEP),_), 
	metadataStream(k,delay,stream(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,STEP),VALUE),
	metadataOutput(METADATA,SYSTEM_SOURCE,SYSTEM_TARGET,VALUE_PROCESSING).


%% Computing map QoS metadata
% fix
metadataMap(k,METADATA,map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP),FIX_VALUE) :- 
	map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP),
	metadataOutput(METADATA,SYSTEM,NODE,fix,FIX_VALUE,MOD_VALUE).

% min
metadataMap(k,METADATA,map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP),FIX_VALUE+VALUE+COUNT*MOD_VALUE) :- 
	map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,min,FIX_VALUE,MOD_VALUE),
	VALUE = #min{D,STEP2,PROVIDER,CONNECTED : 
		connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),CONNECTED,PROVIDER,STEP2),
		metadataElement(k,METADATA,CONNECTED,D), 
		NODE != PROVIDER, 
		STEP2 < STEP}, 
	mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,COUNT),
	COUNT != 0.

% avg
metadataMap(k,METADATA,map(k,SYSTEM,NODE,SOURCE,informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),STEP),FIX_VALUE+VALUE/COUNT+COUNT*MOD_VALUE) :- 
	map(k,SYSTEM,NODE,SOURCE,informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,avg,FIX_VALUE,MOD_VALUE),
	VALUE = #sum{D,STEP2,PROVIDER,CONNECTED : 
		connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),CONNECTED,PROVIDER,STEP2),
		metadataElement(k,METADATA,CONNECTED,D), 
		NODE != PROVIDER, 
		STEP2 < STEP}, 
	mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,COUNT),
	COUNT != 0.


% max
%*
inputMetaMapNode(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),METADATA,VALUE) :- 
	metadataOutput(METADATA,SYSTEM,NODE,max,_,_),
	connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),CONNECTED,_,_),
	metadataElement(k,METADATA,CONNECTED,VALUE).

metadataMap(k,METADATA,map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP),FIX_VALUE+VALUE+COUNT*MOD_VALUE) :- 
	map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D : inputMetaMapNode(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),METADATA,D)}, 
	mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,COUNT),
	COUNT != 0.


metadataMap(k,METADATA,map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP),FIX_VALUE+VALUE+COUNT*MOD_VALUE) :- 
	map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,STEP2,PROVIDER,CONNECTED : 
		connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),CONNECTED,PROVIDER,STEP2),
		metadataElement(k,METADATA,CONNECTED,D), 
		NODE != PROVIDER, 
		STEP2 < STEP}, 
	mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,COUNT),
	COUNT != 0.
*%

-maxMetaMap(k,METADATA,map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP),VALUE) :- 
	map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,max,_,_),
	connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),CONNECTED,_,STEP2),
	metadataElement(k,METADATA,CONNECTED,VALUE),
	connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),CONNECTED2,_,STEP3),
	metadataElement(k,METADATA,CONNECTED2,VALUE2),
	CONNECTED != CONNECTED2,
	STEP2 < STEP,
	STEP3 < STEP,
	VALUE2 > VALUE.
	
maxMetaMap(k,METADATA,map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP),VALUE) :- 
	map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,max,_,_),
	connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),CONNECTED,_,_),
	metadataElement(k,METADATA,CONNECTED,VALUE),
	not -maxMetaMap(k,METADATA,map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP),VALUE).

metadataMap(k,METADATA,map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP),FIX_VALUE+VALUE+COUNT*MOD_VALUE) :- 
	map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,max,FIX_VALUE,MOD_VALUE),
	maxMetaMap(k,METADATA,map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP),VALUE),
	mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,COUNT),
	COUNT != 0.

% no input
metadataMap(k,METADATA,map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP),FIX_VALUE) :- 
	map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,TYPE,FIX_VALUE,MOD_VALUE),
	mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,0),
        TYPE != fix.

% density - special case
metadataMap(k,density,map(k,SYSTEM,NODE,SOURCE,informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),STEP),VALUE+VALUE_SUM) :- 
	map(k,SYSTEM,NODE,SOURCE,informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),STEP), 
	metadataOutput(density,SYSTEM,NODE,sum,_,_),
	VALUE = #count{ ENTITY : connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),stream(k,_,_,_,information(ENTITY,_,_,_),_),_,_)}, 
	VALUE_SUM = #sum{D,STEP2,PROVIDER,CONNECTED : 
		connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),CONNECTED,PROVIDER,STEP2),
		metadataElement(k,density,CONNECTED,D),
		NODE != PROVIDER, 
		STEP2 < STEP
		}, 
	query(k).

metadataElement(k,METADATA,STREAM,D) :- metadataStream(k,METADATA,STREAM,D).
metadataElement(k,METADATA,MAP,D) :- metadataMap(k,METADATA,MAP,D).


%% Speed up
:- metadataMap(k,METADATA,MAP,VALUE), 
   metadataMap(k,METADATA,MAP,VALUE2),
   VALUE != VALUE2.


%% Transfer of maps
% computing QoS metadata	
metadataMap(k,METADATA,map(k,SYSTEM_TARGET,PROVIDER,SOURCE,INFO_TYPE,STEP+1),VALUE) :- 
	mapTransfer(k,SYSTEM_TARGET,map(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO_TYPE,STEP),_), 
	metadataMap(k,METADATA,map(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO_TYPE,STEP),VALUE), 
	not metadataOutput(METADATA,SYSTEM_SOURCE,SYSTEM_TARGET,_).	

metadataMap(k,METADATA,map(k,SYSTEM_TARGET,PROVIDER,SOURCE,INFO_TYPE,STEP+1),VALUE+VALUE_PROCESSING) :-
	mapTransfer(k,SYSTEM_TARGET,map(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO_TYPE,STEP),_), 
	metadataMap(k,delay,map(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO_TYPE,STEP),VALUE),
	metadataOutput(METADATA,SYSTEM_SOURCE,SYSTEM_TARGET,VALUE_PROCESSING).



% checking stream QoS metadata
:~ requiredStream(SYSTEM,INFO), 
   selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO,REQUIRED_VALUE), 
   metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),VALUE), 
   metadata(information,METADATA,maximize),
   REQUIRED_VALUE < VALUE,
   query(k). [1@4,requiredStream]
:~ requiredStream(SYSTEM,INFO), 
   selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO,REQUIRED_VALUE), 
   metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),VALUE),
   metadata(information,METADATA,minimize), 
   REQUIRED_VALUE > VALUE,
   query(k). [1@4,requiredStream]

:~ requiredStream(SYSTEM,INFO), 
   selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO,REQUIRED_VALUE), 
   metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),VALUE), 
   metadata(stream,METADATA,maximize),
   REQUIRED_VALUE < VALUE,
   query(k). [1@4,requiredStream]
:~ requiredStream(SYSTEM,INFO), 
   selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO,REQUIRED_VALUE), 
   metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),VALUE),
   metadata(stream,METADATA,minimize), 
   REQUIRED_VALUE > VALUE,
   query(k). [1@4,requiredStream]



% checking map QoS parameters
:~ selectedMap(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,STEP),
   requiredMetadata(METADATA,SYSTEM,ENTITY_TYPE,SCOPE,REP,ENTITY2,REQUIRED_VALUE), 
   metadataMap(k,METADATA,map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP),VALUE),
   metadata(information,METADATA,maximize),
   REQUIRED_VALUE < VALUE,
   query(k). [1@4,requiredStream]
:~ selectedMap(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,STEP),
   requiredMetadata(METADATA,SYSTEM,ENTITY_TYPE,SCOPE,REP,ENTITY2,REQUIRED_VALUE), 
   metadataMap(k,METADATA,map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP),VALUE),
   metadata(information,METADATA,minimize), 
   REQUIRED_VALUE > VALUE,
   query(k). [1@4,requiredStream]

:~ selectedMap(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,STEP),
   requiredMetadata(METADATA,SYSTEM,ENTITY_TYPE,SCOPE,REP,ENTITY2,REQUIRED_VALUE), 
   metadataMap(k,METADATA,map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP),VALUE),
   metadata(stream,METADATA,maximize),
   REQUIRED_VALUE < VALUE,
   query(k). [1@4,requiredStream]
:~ selectedMap(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,STEP),
   requiredMetadata(METADATA,SYSTEM,ENTITY_TYPE,SCOPE,REP,ENTITY2,REQUIRED_VALUE), 
   metadataMap(k,METADATA,map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP),VALUE),
   metadata(stream,METADATA,minimize), 
   REQUIRED_VALUE > VALUE,
   query(k). [1@4,requiredStream]


% optimizing Stream QoS metadata
%#minimize { VALUE@priorityDelay,INFO,SYSTEM,PROVIDER,SOURCE,STEP : 
%		selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP), 
%		metadataStream(k,delay,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),VALUE), 
%		query(k) }.
%#maximize { VALUE@priorityAccuracy,INFO,SYSTEM,PROVIDER,SOURCE,STEP : 
%		selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP), 
%		metadataStream(k,accuracy,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),VALUE), 
%		query(k) }.



% optimizing QoS metadata
metadataSelected(METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),VALUE) :-
	selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP), 
	metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),VALUE).

metadataSelected(METADATA,map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP),VALUE) :-
	selectedMap(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP), 
	metadataMap(k,METADATA,map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP),VALUE).

#minimize { VALUE@priorityDelay,SELECTED : metadataSelected(delay,SELECTED,VALUE) }.
#maximize { VALUE@priorityAccuracy,SELECTED : metadataSelected(accuracy,SELECTED,VALUE) }.
%#maximize{ 1@4,ENTITY,SYSTEM,NODE,SOURCE,ENTITY_TYPE,ENTITY2,STEP : 
%		selectedMap(k,SYSTEM,NODE,SOURCE,informationType(ENTITY_TYPE,_,_,ENTITY2),STEP), 
%		connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),stream(k,_,_,_,information(ENTITY,_,_,_),_),_,_), query(k) }.

#maximize { VALUE@4,SELECTED : metadataSelected(density,SELECTED,VALUE) }.
%#maximize { VALUE@4,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP : selectedMap(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP), 
%		metadataMap(k,density,map(k,SYSTEM,NODE,SOURCE,INFO_TYPE,STEP),VALUE), query(k) }.








