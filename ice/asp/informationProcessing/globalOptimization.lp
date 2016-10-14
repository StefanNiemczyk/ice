%%%%--------------------------------------------------------------------------------------
%%%%
%%%% Global optimization
%%%%
%%%% Implements a global optimization and propagation of QoS metadata  
%%%%
%%%%--------------------------------------------------------------------------------------
#program query(k,maxHopCount,maxStepCount).

%% computing stream QoS metadata 
inputMetaNode(NODE,METADATA,VALUE) :-
	connectToNode(NODE,stream(k,SYSTEM,PROVIDER,INFO,STEP)),
	metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,INFO,STEP),VALUE).

%fix
metadataStream(k,METADATA,stream(k,SYSTEM,node(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,STEP),FIX_VALUE) :- 
	stream(k,SYSTEM,node(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,fix,FIX_VALUE,_).

%min
metadataStream(k,METADATA,stream(k,SYSTEM,node(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,STEP),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
	stream(k,SYSTEM,node(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,min,FIX_VALUE,MOD_VALUE),
	VALUE = #min{D : inputMetaNode(node(k,SYSTEM,NODE,ENTITY,ENTITY2),METADATA,D)}, 
	nodeMeta(node(k,SYSTEM,NODE,ENTITY,ENTITY2), STEP-1, COUNT),
	COUNT > 0.

%avg
metadataStream(k,METADATA,stream(k,SYSTEM,node(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,STEP),VALUE/COUNT+FIX_VALUE+MOD_VALUE*COUNT) :- 
	stream(k,SYSTEM,node(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,avg,FIX_VALUE,MOD_VALUE),
	VALUE = #sum{D,STEP2,PROVIDER,INFO2 : 
		connectToNode(node(k,SYSTEM,NODE,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,INFO2,STEP2)),
		metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,INFO2,STEP2),D), 
		node(k,SYSTEM,NODE,ENTITY,ENTITY2) != PROVIDER, 
		STEP2 < STEP}, 
	nodeMeta(node(k,SYSTEM,NODE,ENTITY,ENTITY2), STEP-1, COUNT),
	COUNT > 0.

%max
metadataStream(k,METADATA,stream(k,SYSTEM,node(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,STEP),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
	stream(k,SYSTEM,node(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D : inputMetaNode(node(k,SYSTEM,NODE,ENTITY,ENTITY2),METADATA,D)}, 
	nodeMeta(node(k,SYSTEM,NODE,ENTITY,ENTITY2), STEP-1, COUNT),
	COUNT > 0.

%no input
metadataStream(k,METADATA,stream(k,SYSTEM,node(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY,SCOPE,REP,ENTITY2),STEP),FIX_VALUE) :- 
	stream(k,SYSTEM,node(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY,SCOPE,REP,ENTITY2),STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,TYPE,FIX_VALUE,_),
	TYPE != fix,
	nodeMeta(node(k,SYSTEM,NODE,ENTITY,ENTITY2), STEP-1, 0).

% Speed up stream metadata processing
:- metadataStream(k,METADATA,STREAM,VALUE), 
   metadataStream(k,METADATA,STREAM,VALUE2),
   VALUE != VALUE2.


%% Transfer of streams
% computing QoS metadata	
metadataStream(k,METADATA,stream(k,SYSTEM_TARGET,PROVIDER,INFO,STEP+1),VALUE) :- 
	streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,PROVIDER,INFO,STEP),_), 
	metadataStream(k,METADATA,stream(k,SYSTEM_SOURCE,PROVIDER,INFO,STEP),VALUE), 
	not metadataOutput(METADATA,SYSTEM_SOURCE,SYSTEM_TARGET,_).	

metadataStream(k,METADATA,stream(k,SYSTEM_TARGET,PROVIDER,INFO,STEP+1),VALUE+VALUE_PROCESSING) :-
	streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,PROVIDER,INFO,STEP),_), 
	metadataStream(k,METADATA,stream(k,SYSTEM_SOURCE,PROVIDER,INFO,STEP),VALUE),
	metadataOutput(METADATA,SYSTEM_SOURCE,SYSTEM_TARGET,VALUE_PROCESSING).


%% Computing map QoS metadata
% fix
metadataMap(k,METADATA,map(k,SYSTEM,mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP),FIX_VALUE) :- 
	map(k,SYSTEM,mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP),
	metadataOutput(METADATA,SYSTEM,NODE,fix,FIX_VALUE,MOD_VALUE).

% min
metadataMap(k,METADATA,map(k,SYSTEM,mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP),FIX_VALUE+VALUE+COUNT*MOD_VALUE) :- 
	map(k,SYSTEM,mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,min,FIX_VALUE,MOD_VALUE),
	VALUE = #min{D,CONNECTED : 
		connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),CONNECTED),
		metadataElement(k,METADATA,CONNECTED,D)}, 
	mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,COUNT),
	COUNT != 0.

% avg
metadataMap(k,METADATA,map(k,SYSTEM,mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),STEP),FIX_VALUE+VALUE/COUNT+COUNT*MOD_VALUE) :- 
	map(k,SYSTEM,mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,avg,FIX_VALUE,MOD_VALUE),
	VALUE = #sum{D,CONNECTED : 
		connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),CONNECTED),
		metadataElement(k,METADATA,CONNECTED,D)}, 
	mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,COUNT),
	COUNT != 0.

% max
inputMetaMapNode(NODE,METADATA,VALUE) :-
	connectToMap(NODE,CONNECTED),
	metadataElement(k,METADATA,CONNECTED,VALUE).

-maxMetaMap(METADATA,mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),VALUE) :- 
	metadataOutput(METADATA,SYSTEM,NODE,max,_,_),
	inputMetaMapNode(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),METADATA,VALUE),
	inputMetaMapNode(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),METADATA,VALUE2),
	VALUE2 > VALUE.
	
maxMetaMap(METADATA,mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),VALUE) :- 
	metadataOutput(METADATA,SYSTEM,NODE,max,_,_),
	inputMetaMapNode(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),METADATA,VALUE),
	not -maxMetaMap(METADATA,mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),VALUE).

metadataMap(k,METADATA,map(k,SYSTEM,mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP),FIX_VALUE+VALUE+COUNT*MOD_VALUE) :- 
	map(k,SYSTEM,mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,max,FIX_VALUE,MOD_VALUE),
	maxMetaMap(METADATA,mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),VALUE),
	mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,COUNT),
	COUNT != 0.

% no input
metadataMap(k,METADATA,map(k,SYSTEM,mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP),FIX_VALUE) :- 
	map(k,SYSTEM,mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,TYPE,FIX_VALUE,MOD_VALUE),
	mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,0),
        TYPE != fix.

% density - special case
metadataMap(k,density,map(k,SYSTEM,mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),STEP),VALUE+VALUE_SUM) :- 
	map(k,SYSTEM,mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),STEP), 
	metadataOutput(density,SYSTEM,NODE,sum,_,_),
	VALUE = #count{ ENTITY : connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),stream(k,_,_,information(ENTITY,_,_,_),_))}, 
	VALUE_SUM = #sum{D,MAP_NODE,INFO_TYPE2,STEP2 : 
		connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),map(k,SYSTEM,MAP_NODE,INFO_TYPE2,STEP2)),
		metadataMap(k,density,map(k,SYSTEM,MAP_NODE,INFO_TYPE2,STEP2),D)}, 
	query(k).

metadataElement(k,METADATA,STREAM,D) :- metadataStream(k,METADATA,STREAM,D).
metadataElement(k,METADATA,MAP,D) :- metadataMap(k,METADATA,MAP,D).


%% Speed up
:- metadataMap(k,METADATA,MAP,VALUE), 
   metadataMap(k,METADATA,MAP,VALUE2),
   VALUE != VALUE2.


%% Transfer of maps
% computing QoS metadata	
metadataMap(k,METADATA,map(k,SYSTEM_TARGET,PROVIDER,INFO_TYPE,STEP+1),VALUE) :- 
	mapTransfer(k,SYSTEM_TARGET,map(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),_), 
	metadataMap(k,METADATA,map(k,SYSTEM_SOURCE,PROVIDER,INFO_TYPE,STEP),VALUE), 
	not metadataOutput(METADATA,SYSTEM_SOURCE,SYSTEM_TARGET,_).	

metadataMap(k,METADATA,map(k,SYSTEM_TARGET,PROVIDER,INFO_TYPE,STEP+1),VALUE+VALUE_PROCESSING) :-
	mapTransfer(k,SYSTEM_TARGET,map(k,SYSTEM_SOURCE,PROVIDER,INFO_TYPE,STEP),_), 
	metadataMap(k,delay,map(k,SYSTEM_SOURCE,PROVIDER,INFO_TYPE,STEP),VALUE),
	metadataOutput(METADATA,SYSTEM_SOURCE,SYSTEM_TARGET,VALUE_PROCESSING).


% checking stream QoS metadata
:~ requiredStream(SYSTEM,INFO), 
   selectedStream(k,SYSTEM,PROVIDER,INFO,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO,TRESHOLD), 
   metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,INFO,STEP),VALUE), 
   metadata(information,METADATA,maximize),
   TRESHOLD > VALUE,
   query(k). [1@4,requiredStream]
:~ requiredStream(SYSTEM,INFO), 
   selectedStream(k,SYSTEM,PROVIDER,INFO,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO,TRESHOLD), 
   metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,INFO,STEP),VALUE),
   metadata(information,METADATA,minimize), 
   TRESHOLD < VALUE,
   query(k). [1@4,requiredStream]

:~ requiredStream(SYSTEM,INFO), 
   selectedStream(k,SYSTEM,PROVIDER,INFO,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO,TRESHOLD), 
   metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,INFO,STEP),VALUE), 
   metadata(stream,METADATA,maximize),
   TRESHOLD > VALUE,
   query(k). [1@4,requiredStream]
:~ requiredStream(SYSTEM,INFO), 
   selectedStream(k,SYSTEM,PROVIDER,INFO,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO,TRESHOLD), 
   metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,INFO,STEP),VALUE),
   metadata(stream,METADATA,minimize), 
   TRESHOLD < VALUE,
   query(k). [1@4,requiredStream]


% checking map QoS parameters
:~ selectedMap(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO_TYPE,TRESHOLD), 
   metadataMap(k,METADATA,map(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),VALUE),
   metadata(information,METADATA,maximize),
   TRESHOLD > VALUE,
   query(k). [1@4,requiredStream]
:~ selectedMap(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO_TYPE,TRESHOLD), 
   metadataMap(k,METADATA,map(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),VALUE),
   metadata(information,METADATA,minimize),
   TRESHOLD < VALUE,
   query(k). [1@4,requiredStream]

:~ selectedMap(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO_TYPE,TRESHOLD), 
   metadataMap(k,METADATA,map(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),VALUE),
   metadata(map,METADATA,maximize),
   TRESHOLD > VALUE,
   query(k). [1@4,requiredStream]
:~ selectedMap(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO_TYPE,TRESHOLD), 
   metadataMap(k,METADATA,map(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),VALUE),
   metadata(map,METADATA,minimize),
   TRESHOLD < VALUE,
   query(k). [1@4,requiredStream]



% optimizing QoS metadata
metadataSelected(METADATA,stream(k,SYSTEM,PROVIDER,INFO,STEP),VALUE) :-
	selectedStream(k,SYSTEM,PROVIDER,INFO,STEP), 
	metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,INFO,STEP),VALUE).

metadataSelected(METADATA,map(k,SYSTEM,NODE,INFO_TYPE,STEP),VALUE) :-
	selectedMap(k,SYSTEM,NODE,INFO_TYPE,STEP), 
	metadataMap(k,METADATA,map(k,SYSTEM,NODE,INFO_TYPE,STEP),VALUE).

#minimize { VALUE@priorityDelay,SELECTED : metadataSelected(delay,SELECTED,VALUE) }.
#maximize { VALUE@priorityAccuracy,SELECTED : metadataSelected(accuracy,SELECTED,VALUE) }.

#maximize { VALUE@4,SELECTED : metadataSelected(density,SELECTED,VALUE) }.








