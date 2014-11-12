% information/3		:		(entity,scope,representation)
% stream/3		: 		(system,source,information)
% node/3		:		(system,nodeName,entity)
% output/6 		:		(system,node,scope,rep,acc_mode,acc_modifier)
%--------------------------------------------------------------------------------------
% paramters
 #const priorityDelay=3.
 #const priorityAccuracy=3.
 #const priorityTransferCost=2.
 #const priorityCost=1.
 #const minAccuracyValue=0.
 #const maxAccuracyValue=100000.
 

%--------------------------------------------------------------------------------------
#program base.
%#show streamDelay/5.
%#show nodeTemplate/2.
%#show transfer/4.
%#show requiredStream/4.
%#show system/2.

%#show node/4.
%#show stream/5.
%#show connectToNode/6.
%#show streamTransfer/7.
%--------------------------------------------------------------------------------------
#program query(k).

#external query(k).

% preprocessing
%{ stream(k,SYSTEM,SOURCE,INFO) } :- sourceNode(SYSTEM,SOURCE,INFO,_,_,_), query(k).
%streamDelay(k,SYSTEM,SOURCE,INFO,DELAY) :- sourceNode(SYSTEM,SOURCE,INFO,DELAY,_,_), stream(k,SYSTEM,SOURCE,INFO).
%streamAccuracy(k,SYSTEM,SOURCE,INFO,ACCURACY) :- sourceNode(SYSTEM,SOURCE,INFO,_,ACCURACY,_), stream(k,SYSTEM,SOURCE,INFO).
%streamCost(k,SYSTEM,SOURCE,INFO,COST) :- sourceNode(SYSTEM,SOURCE,INFO,_,_,COST), stream(k,SYSTEM,SOURCE,INFO).

%%%
%%% guessing
%%%

%% creating a node
0 { node(k,SYSTEM,NODE,ENTITY) } 1 :- 
	sourceNode(SYSTEM,NODE,ENTITY), 
	entity(ENTITY,_), 
	query(k).
0 { node(k,SYSTEM,NODE,ENTITY) } 1 :- 
	nodeTemplate(SYSTEM,NODE,ENTITY_TYPE), 
	entity(ENTITY,ENTITY_TYPE), 
	query(k).
0 { node(k,SYSTEM,NODE,ENTITY) } 1 :- 
	nodeTemplate(SYSTEM,NODE,any), 
	entity(ENTITY,ENTITY_TYPE), 
	hasScope(ENTITY_TYPE,SCOPE), 
	input(SYSTEM,NODE,SCOPE,_,_,_,_), 
	query(k).

MIN { connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2)) : stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2)), PROVIDER != NODE } MAX :- 
	input(SYSTEM,NODE,SCOPE,REP,ENTITY2,MIN,MAX), 
	node(k,SYSTEM,NODE,ENTITY).

stream(k,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none)) :- 
	node(k,SYSTEM,NODE,ENTITY), 
	output(SYSTEM,NODE,SCOPE,REP).


% computing QoS parameters
%fix
metadataStream(k,METADATA,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none),FIX_VALUE) :- 
	node(k,SYSTEM,NODE,ENTITY), 
	output(SYSTEM,NODE,SCOPE,REP), 
	metadataNode(METADATA,SYSTEM,NODE,fix,FIX_VALUE,MOD_VALUE).

%min
metadataStream(k,METADATA,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
	node(k,SYSTEM,NODE,ENTITY),
	output(SYSTEM,NODE,SCOPE,REP), 
	metadataNode(METADATA,SYSTEM,NODE,min,FIX_VALUE,MOD_VALUE),
	VALUE = #min{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}. 
%	COUNT!=0.
%metadataStream(k,METADATA,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none),FIX_VALUE) :- 
%	node(k,SYSTEM,NODE,ENTITY), 
%	output(SYSTEM,NODE,SCOPE,REP), 
%	metadataNode(METADATA,SYSTEM,NODE,min,FIX_VALUE,MOD_VALUE),
%	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
%	COUNT==0.

%avg
metadataStream(k,METADATA,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none),VALUE/COUNT+FIX_VALUE+MOD_VALUE*COUNT) :- 
	node(k,SYSTEM,NODE,ENTITY), 
	output(SYSTEM,NODE,SCOPE,REP), 
	metadataNode(METADATA,SYSTEM,NODE,avg,FIX_VALUE,MOD_VALUE),
	VALUE = #sum{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
	COUNT!=0.
metadataStream(k,METADATA,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none),FIX_VALUE) :- 
	node(k,SYSTEM,NODE,ENTITY), 
	output(SYSTEM,NODE,SCOPE,REP), 
	metadataNode(METADATA,SYSTEM,NODE,avg,FIX_VALUE,MOD_VALUE),
	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
	COUNT==0.

%max
metadataStream(k,METADATA,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
	node(k,SYSTEM,NODE,ENTITY), 
	output(SYSTEM,NODE,SCOPE,REP), 
	metadataNode(METADATA,SYSTEM,NODE,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)},
	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
	COUNT!=0.
metadataStream(k,METADATA,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none),FIX_VALUE) :- 
	node(k,SYSTEM,NODE,ENTITY), 
	output(SYSTEM,NODE,SCOPE,REP),
	metadataNode(METADATA,SYSTEM,NODE,max,FIX_VALUE,MOD_VALUE),
	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)},
	COUNT==0.



%% transfer a stream between systems
0 { streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,TRANSFER_DELAY,TRANSFER_COST) } 1 :- 
	transfer(SYSTEM_SOURCE,SYSTEM_TARGET,TRANSFER_DELAY,TRANSFER_COST),
	stream(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO), 
	SYSTEM_TARGET != SOURCE.


% computing QoS parameters		
stream(k,SYSTEM_TARGET,PROVIDER,SOURCE,INFO) :- 
	streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,TRANSFER_DELAY,TRANSFER_COST).

% hack to bound the delay computation. max 5 duplicated computations
metadataStream(k,delay,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,TRANSFER_DELAY+ DELAY) :-
	streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,TRANSFER_DELAY,TRANSFER_COST), 
	metadataStream(k,delay,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,DELAY), 
	TRANSFER_DELAY * 5 > DELAY.
metadataStream(k,delay,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,DELAY) :- 
	streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,TRANSFER_DELAY,TRANSFER_COST), 
	metadataStream(k,delay,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,DELAY), 
	TRANSFER_DELAY * 5 < DELAY.

metadataStream(k,METADATA,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,ACCURACY) :- 
	streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,TRANSFER_DELAY,TRANSFER_COST), 
	metadataStream(k,METADATA,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,ACCURACY), 
	METADATA != delay.



%% translate representation
%0 { informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,none,none) } 1 :- 
%	iro(SYSTEM,IRO,any,SCOPE), 
%	entity(ENTITY,ENTITY_TYPE), 
%	hasScope(ENTITY_TYPE,SCOPE),
%	query(k).
%0 { informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,none,none) } 1 :- 
%	iro(SYSTEM,IRO,ENTITY_TYPE,SCOPE),
%	entity(ENTITY,ENTITY_TYPE), 
%	hasScope(ENTITY_TYPE,SCOPE), 
%	query(k).
%0 { informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,ENTITY2,SCOPE2) } 1 :- 
%	iro(SYSTEM,IRO,any,SCOPE,any,SCOPE2), 
%	entity(ENTITY,ENTITY_TYPE), 
%	hasScope(ENTITY_TYPE,SCOPE), 
%	entity(ENTITY2,ENTITY_TYPE2), 
%	hasScope(ENTITY_TYPE2,SCOPE2), 
%	query(k).
%0 { informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,ENTITY2,SCOPE2) } 1 :- 
%	iro(SYSTEM,IRO,ENTITY_TYPE,SCOPE,ENTITY_TYPE2,SCOPE2),
%	entity(ENTITY,ENTITY_TYPE),
%	hasScope(ENTITY_TYPE,SCOPE), 
%	entity(ENTITY2,ENTITY_TYPE2), 
%	hasScope(ENTITY_TYPE2,SCOPE2), 
%	query(k).

0 { informationTranslation(k,SYSTEM,IRO,ENTITY,ENTITY2) } 1 :- 
	iro(SYSTEM,IRO,any), 
	entity(ENTITY,ENTITY_TYPE), 
	hasScope(ENTITY_TYPE,SCOPE), 
	input(SYSTEM,IRO,SCOPE,_,ENTITY2,_,_),
	query(k).
% TODO

MIN { connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2)) : stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2)), PROVIDER != IRO } MAX :- 
	input(SYSTEM,IRO,SCOPE,REP,ENTITY2,MIN,MAX),
	informationTranslation(k,SYSTEM,IRO,ENTITY,ENTITY2).
MIN { connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none)) : stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none)), PROVIDER != IRO } MAX :- 
	input2(SYSTEM,IRO,SCOPE,REP,MIN,MAX), 
	informationTranslation(k,SYSTEM,IRO,_,ENTITY).

stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2)) :- 
	informationTranslation(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP).


% computing QoS parameters
%fix
metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),FIX_VALUE) :- 
	informationTranslation(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP), 
	metadataIro(METADATA,SYSTEM,IRO,fix,FIX_VALUE,MOD_VALUE).

%min
metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
	informationTranslation(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP), 
	metadataIro(METADATA,SYSTEM,IRO,min,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}.
%	COUNT!=0.
%metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),FIX_VALUE) :- 
%	informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,ENTITY2,_), 
%	output(SYSTEM,IRO,SCOPE2,REP), 
%	metadataIro(METADATA,SYSTEM,IRO,min,FIX_VALUE,MOD_VALUE),
%	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)},
%	COUNT==0.

%avg
metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),VALUE/COUNT+FIX_VALUE+MOD_VALUE*COUNT) :- 
	informationTranslation(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP), 
	metadataIro(METADATA,SYSTEM,IRO,avg,FIX_VALUE,MOD_VALUE),
	VALUE = #sum{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
	COUNT!=0.
metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),FIX_VALUE) :- 
	informationTranslation(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP),
	metadataIro(METADATA,SYSTEM,IRO,avg,FIX_VALUE,MOD_VALUE),
	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
	COUNT==0.

%max
metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),VALUE+FIX_VALUE+MOD_VALUE*COUNT) :- 
	informationTranslation(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP),
	metadataIro(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)},
	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
	COUNT!=0.
metadataStream(k,METADATA,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2),FIX_VALUE) :- 
	informationTranslation(k,SYSTEM,IRO,ENTITY,ENTITY2), 
	output(SYSTEM,IRO,SCOPE,REP), metadataIro(METADATA,SYSTEM,IRO,max,FIX_VALUE,MOD_VALUE),
	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
	COUNT==0.



%% extract from representation
0 { informationExtrection(k,SYSTEM,PROVIDER,SOURCE,SCOPE,REP1,information(ENTITY,SUB_SCOPE,REP2,none)) } 1 :- 
	stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP1,none)), 
	hasDimension(REP1,SUB_SCOPE,_,_), 
	hasRepresentation(SUB_SCOPE,REP2), 
	query(k).
stream(k,SYSTEM,PROVIDER,SOURCE,INFO) :- 
	informationExtrection(k,SYSTEM,PROVIDER,SOURCE,SCOPE,REP1,INFO).


% computing QoS parameters	
metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SUB_SCOPE,REP2,ENTITY2),ACCURACY) :- 
	informationExtrection(k,SYSTEM,PROVIDER,SOURCE,SCOPE,REP1,information(ENTITY,SUB_SCOPE,REP2,ENTITY2)),
	metadataStream(k,METADATA,SYSTEM_SOURCE,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP1,ENTITY2),ACCURACY).



%% creating a map
0 { mapNode(k,SYSTEM,NODE,ENTITY_TYPE) } 1 :- 
	mapNodeTemplate(SYSTEM,NODE,ENTITY_TYPE),
	query(k).

MIN { connectToMap(k,SYSTEM,NODE,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2)) : stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2)), entity(ENTITY,ENTITY_TYPE) } MAX :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE), 
	input(SYSTEM,NODE,SCOPE,REPRESENTATION,ENTITY2,MIN,MAX).

map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2) :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE), 
	outputMap(SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2).


% computing QoS parameters
% fix
metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,FIX_VALUE) :- 
	map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2),
	metadataMap(METADATA,SYSTEM,MAP,fix,FIX_VALUE,MOD_VALUE).

% min
metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,FIX_VALUE+VALUE+COUNT*MOD_VALUE) :- 
	map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2), 
	metadataMap(METADATA,SYSTEM,MAP,min,FIX_VALUE,MOD_VALUE),
	VALUE = #min{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)},
	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}.

%avg
metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,RESULT) :- 
	map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2), 
	metadataMap(METADATA,SYSTEM,MAP,avg,FIX_VALUE,MOD_VALUE),
	VALUE = #sum{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)},
	RESULT = VALUE/COUNT+FIX_VALUE+COUNT*MOD_VALUE, 
	COUNT!=0.
metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,FIX_VALUE) :- 
	map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2), 
	metadataMap(METADATA,SYSTEM,MAP,avg,FIX_VALUE,MOD_VALUE), 
	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
	COUNT==0.

%max
metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,FIX_VALUE) :- 
	map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2), 
	metadataMap(METADATA,SYSTEM,MAP,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), 
	metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
	VALUE==#infimum.
metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,FIX_VALUE+VALUE+COUNT*MOD_VALUE) :- 
	map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2), 
	metadataMap(METADATA,SYSTEM,MAP,max,FIX_VALUE,MOD_VALUE),
	VALUE = #max{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)},
	COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
	VALUE!=#infimum.



%%%
%%% checking
%%% soft optimization contraint instead of a hard constraint to enable partial solutions
%%%

metadata(accuracy,maximize).
metadata(delay,minimize).

%% streams
0 { selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO) : stream(k,SYSTEM,PROVIDER,SOURCE,INFO) } 1 :- requiredStream(SYSTEM,INFO), query(k).
:~ requiredStream(SYSTEM,INFO), not selectedStream(k,SYSTEM,_,_,INFO), query(k). [1@5,requiredStream]

% checking QoS parameters
:~ requiredStream(SYSTEM,INFO), 
   selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO),
   requiredMetadata(METADATA,SYSTEM,INFO,REQUIRED_VALUE), 
   metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,VALUE), 
   metadata(METADATA,maximize),
   REQUIRED_VALUE < VALUE,query(k). [1@4,requiredStream]
:~ requiredStream(SYSTEM,INFO), 
   selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO),
   requiredMetadata(METADATA,SYSTEM,INFO,REQUIRED_VALUE), 
   metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,VALUE),
   metadata(METADATA,minimize), 
   REQUIRED_VALUE > VALUE,query(k). [1@4,requiredStream]


%% maps
0 { selectedMap(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2) : map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2) } 1 :- 
	requiredMap(SYSTEM,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2), query(k).
:~ requiredMap(SYSTEM,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2), not selectedMap(k,SYSTEM,_,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2), query(k). [1@5,requiredMap]

% checking QoS parameters
:~ selectedMap(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2),
   requiredMetadata(METADATA,SYSTEM,ENTITY_TYPE,SCOPE,REP,ENTITY2,REQUIRED_VALUE), 
   metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,VALUE), 
   metadata(METADATA,maximize),
   REQUIRED_VALUE < VALUE,query(k). [1@4,requiredStream]
:~    selectedMap(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2),
   requiredMetadata(METADATA,SYSTEM,ENTITY_TYPE,SCOPE,REP,ENTITY2,REQUIRED_VALUE), 
   metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,VALUE), 
   metadata(METADATA,minimize), 
   REQUIRED_VALUE > VALUE,query(k). [1@4,requiredStream]


% checking point budget of systems
:- system(SYSTEM,POINTS), SUM_STREAMS = #sum{COST,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO : streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,_,COST)}, 
		SUM_NODES = #sum{COST,SYSTEM,NODE,ENTITY : node(k,SYSTEM,NODE,ENTITY), nodeCost(SYSTEM,NODE,COST)}, (SUM_NODES + SUM_STREAMS) > POINTS, query(k).



%%%
%%% optimize
%%%

% stream QoS
#minimize { DELAY@priorityDelay,SYSTEM,PROVIDER,SOURCE,INFO : selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO), metadataStream(k,delay,SYSTEM,PROVIDER,SOURCE,INFO,DELAY), query(k) }.
#maximize { ACCURACY@priorityAccuracy,SYSTEM,PROVIDER,SOURCE,INFO : selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO), metadataStream(k,accuracy,SYSTEM,PROVIDER,SOURCE,INFO,ACCURACY) }.

% map QoS
#minimize { DELAY@priorityDelay,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2 : selectedMap(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2), 
		metadataMap(k,delay,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,DELAY), query(k) }.
#maximize { VALUE@priorityAccuracy,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2 : selectedMap(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2), 
		metadataMap(k,accuracy,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,VALUE), query(k) }.

% transfer cost
#minimize { TRANSFER_COST@priorityTransferCost,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO : streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,_,TRANSFER_COST), query(k) }.

% cost
sumCost(k,SUM_NODES + SUM_TRANSLATIONS + SUM_EXTRACTIONS) :- 
	SUM_NODES = #sum{COST,SYSTEM,NODE,ENTITY : node(k,SYSTEM,NODE,ENTITY), nodeCost(SYSTEM,NODE,COST)}, 
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

output(system,node,scope,rep).

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









