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
%#show node/5.
%#show before/4.
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
%%% 
%%%

% computing theoretical nodes
tnode(k,SYSTEM,NODE,ENTITY,none) :- 
	sourceNode(SYSTEM,NODE,ENTITY), 
	entity(ENTITY,_), 
	query(k).
tnode(k,SYSTEM,NODE,ENTITY,none) :- 
	nodeTemplate(SYSTEM,NODE,ENTITY_TYPE), 
	entity(ENTITY,ENTITY_TYPE), 
	query(k),
	stream(k,SYSTEM,_,_,information(ENTITY,SCOPE2,REP2,none),_) : input(SYSTEM,NODE,SCOPE2,REP2,MIN,_).
tnode(k,SYSTEM,NODE,ENTITY,none) :- 
	nodeTemplate(SYSTEM,NODE,any), 
	entity(ENTITY,ENTITY_TYPE), 
	hasScope(ENTITY_TYPE,SCOPE), 
	input(SYSTEM,NODE,SCOPE,_,_,_,_),
	query(k).
 
tnode(k,SYSTEM,IRO,ENTITY,ENTITY2) :- 
	iro(SYSTEM,IRO,any,any), 
	entity(ENTITY,ENTITY_TYPE), 
	hasScope(ENTITY_TYPE,SCOPE), 
	input(SYSTEM,IRO,SCOPE,_,_,_,_),
	entity(ENTITY2,ENTITY_TYPE2), 
	hasScope(ENTITY_TYPE2,SCOPE2), 
	input2(SYSTEM,IRO,SCOPE2,_,_,_,_),
	query(k).
tnode(k,SYSTEM,IRO,ENTITY,none) :- 
	iro(SYSTEM,IRO,any,none), 
	entity(ENTITY,ENTITY_TYPE), 
	hasScope(ENTITY_TYPE,SCOPE), 
	input(SYSTEM,IRO,SCOPE,_,_,_,_),
	query(k).


% computing theoretical streams created by nodes
tstream(k,SYSTEM,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY,SCOPE,REP,ENTITY2)) :- 
	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2), 
%	heuristic(k,SYSTEM,NODE,C),
	ENTITY2 != none,
	ENTITY2 != any,
	output(SYSTEM,NODE,SCOPE,REP,ENTITY2).

tstream(k,SYSTEM,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY,SCOPE,REP,none)) :- 
	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),
%	heuristic(k,SYSTEM,NODE,C),
	output(SYSTEM,NODE,SCOPE,REP,none).

tstream(k,SYSTEM,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY,SCOPE,REP,ENTITY2)) :- 
	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2), 
%	heuristic(k,SYSTEM,NODE,C),
	output(SYSTEM,NODE,SCOPE,REP,any).


% computing theoretical inputs for nodes
tinput(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY,SCOPE,REP,ENTITY2),MIN,MAX) :- 
	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2), input(SYSTEM,NODE,SCOPE,REP,ENTITY2,MIN,MAX), ENTITY2 != none, ENTITY2 != any.

tinput(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY,SCOPE,REP,none),MIN,MAX) :- 
	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2), input(SYSTEM,NODE,SCOPE,REP,none,MIN,MAX).

tinput(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY,SCOPE,REP,ENTITY2),MIN,MAX) :- 
	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2), input(SYSTEM,NODE,SCOPE,REP,any,MIN,MAX).


tinput(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY2,SCOPE,REP,ENTITY),MIN,MAX) :- 
	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2), input2(SYSTEM,NODE,SCOPE,REP,ENTITY,MIN,MAX), ENTITY2 != none, ENTITY2 != any.

tinput(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY2,SCOPE,REP,none),MIN,MAX) :- 
	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2), input2(SYSTEM,NODE,SCOPE,REP,none,MIN,MAX).

tinput(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY2,SCOPE,REP,ENTITY),MIN,MAX) :- 
	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2), input2(SYSTEM,NODE,SCOPE,REP,any,MIN,MAX).


% compute number of inputs for each node
nodeCount(k,SYSTEM,NODE, MINV+MINV2, MAXV+MAXV2) :- 
	sourceNode(SYSTEM,NODE,_), 
	MINV = #sum{MIN,SCOPE,REP,ENT2 : input(SYSTEM,NODE,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV = #sum{MAX,SCOPE,REP,ENT2 : input(SYSTEM,NODE,SCOPE,REP,ENT2,_,MAX)}, 
	MINV2 = #sum{MIN,SCOPE,REP,ENT2 : input2(SYSTEM,NODE,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV2 = #sum{MAX,SCOPE,REP,ENT2 : input2(SYSTEM,NODE,SCOPE,REP,ENT2,_,MAX)}.

nodeCount(k,SYSTEM,NODE, MINV+MINV2, MAXV+MAXV2) :- 
	nodeTemplate(SYSTEM,NODE,_), 
	MINV = #sum{MIN,SCOPE,REP,ENT2 : input(SYSTEM,NODE,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV = #sum{MAX,SCOPE,REP,ENT2 : input(SYSTEM,NODE,SCOPE,REP,ENT2,_,MAX)}, 
	MINV2 = #sum{MIN,SCOPE,REP,ENT2 : input2(SYSTEM,NODE,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV2 = #sum{MAX,SCOPE,REP,ENT2 : input2(SYSTEM,NODE,SCOPE,REP,ENT2,_,MAX)}.

nodeCount(k,SYSTEM,IRO, MINV+MINV2, MAXV+MAXV2) :- 
	iro(SYSTEM,IRO,_,_), 
	MINV = #sum{MIN,SCOPE,REP,ENT2 : input(SYSTEM,IRO,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV = #sum{MAX,SCOPE,REP,ENT2 : input(SYSTEM,IRO,SCOPE,REP,ENT2,_,MAX)}, 
	MINV2 = #sum{MIN,SCOPE,REP,ENT2 : input2(SYSTEM,IRO,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV2 = #sum{MAX,SCOPE,REP,ENT2 : input2(SYSTEM,IRO,SCOPE,REP,ENT2,_,MAX)}.


% connect to node
MIN { connectToNode(node(k,SYSTEM,NODE,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP)) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP), 
		STEP < maxStepCount,
		PROVIDER != NODE } MAX :- 
	node(k,SYSTEM,NODE,ENTITY,ENTITY2),
	tinput(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,MIN,MAX).


% computing meta inforamtion of a node
nodeMeta(node(k,SYSTEM,NODE,ENTITY,ENTITY2), STEP_MAX, COUNT) :- 
	node(k,SYSTEM,NODE,ENTITY,ENTITY2),
	nodeCount(k,SYSTEM,NODE,MIN,MAX),
	MIN != MAX,
	STEP_MAX = #max{STEP : connectToNode(node(k,SYSTEM,NODE,ENTITY,ENTITY2), stream(k,_,_,_,_,STEP))},
	COUNT = #count{PROVIDER,SOURCE,INFO,STEP2 : connectToNode(node(k,SYSTEM,NODE,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2)), 
		PROVIDER != NODE, 
		STEP2 < STEP_MAX+1},
	COUNT > MIN-1,
	COUNT < MAX+1,
	STEP_MAX != #inf.

nodeMeta(node(k,SYSTEM,IRO,ENTITY,ENTITY2), STEP_MAX, COUNT) :- 
	node(k,SYSTEM,IRO,ENTITY,ENTITY2),
	STEP_MAX = #max{STEP : connectToNode(node(k,SYSTEM,IRO,ENTITY,ENTITY2), stream(k,_,_,_,_,STEP))},
	nodeCount(k,SYSTEM,IRO,COUNT,COUNT),
	COUNT > 0;
	STEP_MAX != #inf.

nodeMeta(node(k,SYSTEM,NODE,ENTITY,ENTITY2), 0, 0) :- 
	node(k,SYSTEM,NODE,ENTITY,ENTITY2),
	not connectToNode(node(k,SYSTEM,NODE,ENTITY,ENTITY2), _),
	nodeCount(k,SYSTEM,NODE,0,MAX).

% create output of nodes
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


% Speed up ASP
:- stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP2), 
   stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP), 
   STEP != STEP2.


% Information that could be used
requiredInformation(k,SYSTEM,INFO) :- 
	requiredStream(SYSTEM,INFO).

requiredInformation(k,SYSTEM,INFO) :- 
	tinput(tnode(k,SYSTEM,_,_,_),INFO,_,_).

requiredInformation(k,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2)) :- 
	tmapInput(tmapNode(k,SYSTEM,_,ENTITY_TYPE,ENTITY2),SCOPE,REP,_,_),
	entity(ENTITY,ENTITY_TYPE).


%% transfer a stream between systems
tstreamTransfer(SYSTEM_TARGET,tstream(k,SYSTEM_SOURCE,TNODE,INFO)) :-
	requiredInformation(k,SYSTEM_TARGET,INFO),
	transfer(SYSTEM_SOURCE,SYSTEM_TARGET,_,_),
	tstream(k,SYSTEM_SOURCE,TNODE,INFO),
	SYSTEM_TARGET != SYSTEM_SOURCE.

tstream(k,SYSTEM_TARGET,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY,SCOPE,REP,ENTITY2)) :- 
	tstreamTransfer(SYSTEM_TARGET,tstream(k,SYSTEM,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY,SCOPE,REP,ENTITY2))).
	
stream(k,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,STEP+1) :- 
	streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,STEP),TRANSFER_DELAY,TRANSFER_COST,_).

:- streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE1,PROVIDER,SOURCE,INFO,_),_,_,_), 
   streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE2,PROVIDER,SOURCE,INFO,_),_,_,_), 
   SYSTEM_SOURCE1 != SYSTEM_SOURCE2.


%% creating a map
tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,none) :- 
	mapNodeTemplate(SYSTEM,NODE,ENTITY_TYPE),
	query(k).

% computing theoretical maps
tmap(k,SYSTEM,tnode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),ENTITY_TYPE,SCOPE,REP,ENTITY2) :- 
	tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	outputMap(SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2).

% computing theoretical inputs for maps
tmapInput(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),SCOPE,REP,MIN,MAX) :- 
	tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	input(SYSTEM,NODE,SCOPE,REP,ENTITY2,MIN,MAX).

% inputs
MIN { connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none),STEP)) : 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none),STEP), 
		entity(ENTITY,ENTITY_TYPE), 
		STEP < maxStepCount,
		NODE != PROVIDER } MAX :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	tmapInput(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),SCOPE,REP,MIN,MAX).


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
	MIN != MAX,
	COUNT > MIN-1,
	COUNT < MAX+1,
	STEP_MAX != #inf.

mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), STEP_MAX, COUNT) :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),
	STEP_MAX = #max{STEP : connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), stream(k,_,_,_,_,STEP))},
	mapCount(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), COUNT, COUNT),
	STEP_MAX != #inf.

mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 0, 0) :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),
	STEP_MAX = #max{STEP : connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), stream(k,_,_,_,_,STEP))},
	mapCount(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 0, MAX),
	STEP_MAX == #inf.


map(k,SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,STEP_MAX+1) :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	outputMap(SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2),
	mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), STEP_MAX, COUNT),
	ENTITY2 != any.



%%%
%%% Checking
%%% soft optimization contraint instead of a hard constraint to enable partial solutions
%%%

%% streams
0 { selectedStream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP) : stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP) } 1 :- 
	requiredStream(SYSTEM,INFO), query(k).	
%:- requiredStream(SYSTEM,INFO), not selectedStream(k,SYSTEM,_,_,INFO,_), query(k).

:~ requiredStream(SYSTEM,INFO), not selectedStream(k,SYSTEM,_,_,INFO,_), query(k). [1@5,requiredStream]
%#minimize { 1@5,SYSTEM,INFO :  requiredStream(SYSTEM,INFO), not selectedStream(k,SYSTEM,_,_,INFO,_), query(k) }.
%??? why?


%% maps
0 { selectedMap(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,STEP) : map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,STEP) } 1 :- 
	requiredMap(SYSTEM,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2), query(k).
:~ requiredMap(SYSTEM,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2), not selectedMap(k,SYSTEM,_,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2,_), query(k). [1@5,requiredMap]


% checking point budget of systems
:- system(SYSTEM,POINTS), SUM_STREAMS = #sum{COST,SYSTEM,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,STEP : streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),_,COST,_)}, 
		SUM_NODES = #sum{COST,SYSTEM,NODE,ENTITY,ENTITY2 : node(k,SYSTEM,NODE,ENTITY,ENTITY2), nodeCost(SYSTEM,NODE,COST)}, (SUM_NODES + SUM_STREAMS) > POINTS, query(k).


%%%
%%% Optimization
%%% QoS parameters of selected streams and maps are optimized
%%%

% transfer cost
%sumTransfer(k,SUM) :- SUM = #sum{ TRANSFER_COST,SYSTEM,STREAM : streamTransfer(k,SYSTEM,STREAM,_,TRANSFER_COST,_), query(k) }.
%#minimize { SUM@priorityTransferCost : sumTransfer(k,SUM), query(k) }.
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









