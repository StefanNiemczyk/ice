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
	metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,INFO,STEP),VALUE),
	VALUE < 1000, VALUE > -1.
inputMetaNode(NODE,METADATA,1000) :-
	connectToNode(NODE,stream(k,SYSTEM,PROVIDER,INFO,STEP)),
	metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,INFO,STEP),VALUE),
	VALUE > 1000.
inputMetaNode(NODE,METADATA,0) :-
	connectToNode(NODE,stream(k,SYSTEM,PROVIDER,INFO,STEP)),
	metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,INFO,STEP),VALUE),
	VALUE < 0.

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


%% Computing set QoS metadata
inputMetaSetNode(NODE,METADATA,VALUE) :-
	connectToSet(NODE,CONNECTED),
	metadataElement(k,METADATA,CONNECTED,VALUE),
	VALUE < 1000, VALUE > -1.
inputMetaSetNode(NODE,METADATA,1000) :-
	connectToSet(NODE,CONNECTED),
	metadataElement(k,METADATA,CONNECTED,VALUE),
	VALUE > 1000.
inputMetaSetNode(NODE,METADATA,0) :-
	connectToSet(NODE,CONNECTED),
	metadataElement(k,METADATA,CONNECTED,VALUE),
	VALUE < 0.

% fix
metadataSet(k,METADATA,set(k,SYSTEM,setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP),FIX_VALUE) :- 
	set(k,SYSTEM,setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP),
	metadataOutput(METADATA,SYSTEM,NODE,fix,FIX_VALUE,MOD_VALUE).

% min
metadataSet(k,METADATA,set(k,SYSTEM,setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP),FIX_VALUE+VALUE+COUNT*MOD_VALUE) :- 
	set(k,SYSTEM,setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,min,FIX_VALUE,MOD_VALUE),
	VALUE = #min{D : inputMetaSetNode(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),METADATA,D)}, 
	setMeta(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,COUNT),
	COUNT != 0.

% avg
metadataSet(k,METADATA,set(k,SYSTEM,setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),STEP),FIX_VALUE+VALUE/COUNT+COUNT*MOD_VALUE) :- 
	set(k,SYSTEM,setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,avg,FIX_VALUE,MOD_VALUE),
	VALUE = #sum{D,CONNECTED : 
		connectToSet(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),CONNECTED),
		metadataElement(k,METADATA,CONNECTED,D)}, 
	setMeta(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,COUNT),
	COUNT != 0.

% max
metadataSet(k,METADATA,set(k,SYSTEM,setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP),FIX_VALUE+VALUE+COUNT*MOD_VALUE) :- 
	set(k,SYSTEM,setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D : inputMetaSetNode(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),METADATA,D)}, 
	setMeta(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,COUNT),
	VALUE < 1000,
	COUNT != 0.

% no input
metadataSet(k,METADATA,set(k,SYSTEM,setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP),FIX_VALUE) :- 
	set(k,SYSTEM,setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP), 
	metadataOutput(METADATA,SYSTEM,NODE,TYPE,FIX_VALUE,MOD_VALUE),
	setMeta(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP-1,0),
        TYPE != fix.

% density - special case
metadataSet(k,density,set(k,SYSTEM,setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),STEP),VALUE+VALUE_SUM) :- 
	set(k,SYSTEM,setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),STEP), 
	metadataOutput(density,SYSTEM,NODE,sum,_,_),
	VALUE = #count{ ENTITY : connectToSet(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),stream(k,_,_,information(ENTITY,_,_,_),_))}, 
	VALUE_SUM = #sum{D,SET_NODE,INFO_TYPE2,STEP2 : 
		connectToSet(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),set(k,SYSTEM,SET_NODE,INFO_TYPE2,STEP2)),
		metadataSet(k,density,set(k,SYSTEM,SET_NODE,INFO_TYPE2,STEP2),D)}, 
	VALUE_SUM < 1000,
	query(k).

metadataElement(k,METADATA,STREAM,D) :- metadataStream(k,METADATA,STREAM,D).
metadataElement(k,METADATA,SET,D) :- metadataSet(k,METADATA,SET,D).


%% Speed up
:- metadataSet(k,METADATA,SET,VALUE), 
   metadataSet(k,METADATA,SET,VALUE2),
   VALUE != VALUE2.


%% Transfer of sets
% computing QoS metadata	
metadataSet(k,METADATA,set(k,SYSTEM_TARGET,PROVIDER,INFO_TYPE,STEP+1),VALUE) :- 
	setTransfer(k,SYSTEM_TARGET,set(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),_), 
	metadataSet(k,METADATA,set(k,SYSTEM_SOURCE,PROVIDER,INFO_TYPE,STEP),VALUE), 
	not metadataOutput(METADATA,SYSTEM_SOURCE,SYSTEM_TARGET,_).	

metadataSet(k,METADATA,set(k,SYSTEM_TARGET,PROVIDER,INFO_TYPE,STEP+1),VALUE+VALUE_PROCESSING) :-
	setTransfer(k,SYSTEM_TARGET,set(k,SYSTEM_SOURCE,PROVIDER,INFO_TYPE,STEP),_), 
	metadataSet(k,delay,set(k,SYSTEM_SOURCE,PROVIDER,INFO_TYPE,STEP),VALUE),
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


% checking set QoS parameters
:~ selectedSet(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO_TYPE,TRESHOLD), 
   metadataSet(k,METADATA,set(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),VALUE),
   metadata(information,METADATA,maximize),
   TRESHOLD > VALUE,
   query(k). [1@4,requiredStream]
:~ selectedSet(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO_TYPE,TRESHOLD), 
   metadataSet(k,METADATA,set(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),VALUE),
   metadata(information,METADATA,minimize),
   TRESHOLD < VALUE,
   query(k). [1@4,requiredStream]

:~ selectedSet(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO_TYPE,TRESHOLD), 
   metadataSet(k,METADATA,set(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),VALUE),
   metadata(set,METADATA,maximize),
   TRESHOLD > VALUE,
   query(k). [1@4,requiredStream]
:~ selectedSet(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),
   requiredMetadata(METADATA,SYSTEM,INFO_TYPE,TRESHOLD), 
   metadataSet(k,METADATA,set(k,SYSTEM,PROVIDER,INFO_TYPE,STEP),VALUE),
   metadata(set,METADATA,minimize),
   TRESHOLD < VALUE,
   query(k). [1@4,requiredStream]



% optimizing QoS metadata
metadataSelected(METADATA,stream(k,SYSTEM,PROVIDER,INFO,STEP),VALUE) :-
	selectedStream(k,SYSTEM,PROVIDER,INFO,STEP), 
	metadataStream(k,METADATA,stream(k,SYSTEM,PROVIDER,INFO,STEP),VALUE).

metadataSelected(METADATA,set(k,SYSTEM,NODE,INFO_TYPE,STEP),VALUE) :-
	selectedSet(k,SYSTEM,NODE,INFO_TYPE,STEP), 
	metadataSet(k,METADATA,set(k,SYSTEM,NODE,INFO_TYPE,STEP),VALUE).

#minimize { VALUE@priorityDelay,SELECTED : metadataSelected(delay,SELECTED,VALUE) }.
#maximize { VALUE@priorityAccuracy,SELECTED : metadataSelected(accuracy,SELECTED,VALUE) }.

#maximize { VALUE@4,SELECTED : metadataSelected(density,SELECTED,VALUE) }.








