%--------------------------------------------------------------------------------------
% paramters
#const priorityDelay=3.
#const priorityAccuracy=3.
#const priorityTransferCost=2.
#const priorityCost=1.
 

%--------------------------------------------------------------------------------------
#program base.

metadata(information,accuracy,maximize).
metadata(information,delay,minimize).
metadata(map,density,maximize).
metadata(processing,cost,minimize).


%--------------------------------------------------------------------------------------
#program query(k,maxHopCount,maxStepCount).

#external query(k).

%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% stream guessing
%%% 
%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% computing theoretical nodes
tnode(k,SYSTEM,NODE,ENTITY,none) :- 
	sourceNode(SYSTEM,NODE,ENTITY), 
	entity(ENTITY,_), 
	query(k).
tnode(k,SYSTEM,NODE,ENTITY,none) :- 
	nodeTemplate(SYSTEM,NODE,ENTITY_TYPE), 
	entity(ENTITY,ENTITY_TYPE),
	hasScope(ENTITY_TYPE,SCOPE), 
	input(SYSTEM,NODE,SCOPE,_,_,_), 
	query(k).
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

% identify special nodes
snode(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2)) :-
	tstream(k,SYSTEM,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO),
	tinput(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,_,_).
nnode(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2)) :-
	tstream(k,SYSTEM,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO),
	tinput(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO2,_,_),
	INFO != INFO2.
nnode(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2)) :-
	sourceNode(SYSTEM,NODE,_),
	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2).
	

% computing theoretical streams created by nodes
tstream(k,SYSTEM,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY,SCOPE,REP,ENTITY2)) :- 
	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2), 
	ENTITY2 != none,
	ENTITY2 != any,
	output(SYSTEM,NODE,SCOPE,REP,ENTITY2).

tstream(k,SYSTEM,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY,SCOPE,REP,none)) :- 
	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),
	output(SYSTEM,NODE,SCOPE,REP,none).

tstream(k,SYSTEM,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY,SCOPE,REP,ENTITY2)) :- 
	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2), 
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
tnodeCount(k,SYSTEM,NODE,MINV+MINV2,MAXV+MAXV2) :- 
	sourceNode(SYSTEM,NODE,_), 
	MINV = #sum{MIN,SCOPE,REP,ENT2 : input(SYSTEM,NODE,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV = #sum{MAX,SCOPE,REP,ENT2 : input(SYSTEM,NODE,SCOPE,REP,ENT2,_,MAX)}, 
	MINV2 = #sum{MIN,SCOPE,REP,ENT2 : input2(SYSTEM,NODE,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV2 = #sum{MAX,SCOPE,REP,ENT2 : input2(SYSTEM,NODE,SCOPE,REP,ENT2,_,MAX)}.

tnodeCount(k,SYSTEM,NODE,MINV+MINV2,MAXV+MAXV2) :- 
	nodeTemplate(SYSTEM,NODE,_), 
	MINV = #sum{MIN,SCOPE,REP,ENT2 : input(SYSTEM,NODE,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV = #sum{MAX,SCOPE,REP,ENT2 : input(SYSTEM,NODE,SCOPE,REP,ENT2,_,MAX)}, 
	MINV2 = #sum{MIN,SCOPE,REP,ENT2 : input2(SYSTEM,NODE,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV2 = #sum{MAX,SCOPE,REP,ENT2 : input2(SYSTEM,NODE,SCOPE,REP,ENT2,_,MAX)}.

tnodeCount(k,SYSTEM,IRO,MINV+MINV2,MAXV+MAXV2) :- 
	iro(SYSTEM,IRO,_,_), 
	MINV = #sum{MIN,SCOPE,REP,ENT2 : input(SYSTEM,IRO,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV = #sum{MAX,SCOPE,REP,ENT2 : input(SYSTEM,IRO,SCOPE,REP,ENT2,_,MAX)}, 
	MINV2 = #sum{MIN,SCOPE,REP,ENT2 : input2(SYSTEM,IRO,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV2 = #sum{MAX,SCOPE,REP,ENT2 : input2(SYSTEM,IRO,SCOPE,REP,ENT2,_,MAX)}.

nodeCount(node(k,SYSTEM,NODE,ENTITY,ENTITY2),MIN,MAX) :-
	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),
	tnodeCount(k,SYSTEM,NODE,MIN,MAX).
	

% connect to node
connectToNode(node(k,SYSTEM,NODE,ENTITY,ENTITY2), stream(k,SYSTEM,node(k,SOURCE,PROVIDER,ENTITY3,ENTITY4),INFO,STEP)) :-
	STEP < maxStepCount,
	PROVIDER != NODE,
	before(tnode(k,SOURCE,PROVIDER,ENTITY3,ENTITY4),tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,S), 
	stream(k,SYSTEM,node(k,SOURCE,PROVIDER,ENTITY3,ENTITY4),INFO,STEP). 


% computing meta inforamtion of a node
-maxStepNode(NODE,STEP) :-
	connectToNode(NODE,stream(k,_,_,_,STEP)),
	connectToNode(NODE,stream(k,_,_,_,STEP2)),
	STEP2 > STEP.

maxStepNode(NODE,STEP) :-
	connectToNode(NODE,stream(k,_,_,_,STEP)),
	not -maxStepNode(NODE,STEP).

nodeMeta(node(k,SYSTEM,NODE,ENTITY,ENTITY2),STEP_MAX,COUNT) :- 
	nodeCount(node(k,SYSTEM,NODE,ENTITY,ENTITY2),MIN,MAX),
	MIN != MAX,
	maxStepNode(node(k,SYSTEM,NODE,ENTITY,ENTITY2),STEP_MAX),
	COUNT = #count{STREAM : connectToNode(node(k,SYSTEM,NODE,ENTITY,ENTITY2), STREAM)},
	COUNT < MAX + 1,
	COUNT > MIN - 1.

nodeMeta(node(k,SYSTEM,NODE,ENTITY,ENTITY2), STEP_MAX, COUNT) :- 
	maxStepNode(node(k,SYSTEM,NODE,ENTITY,ENTITY2),STEP_MAX),
	%STEP_MAX = #max{STEP : connectToNode(node(k,SYSTEM,NODE,ENTITY,ENTITY2), stream(_,_,_,_,STEP))},
	nodeCount(node(k,SYSTEM,NODE,ENTITY,ENTITY2),COUNT,COUNT),
	STEP_MAX < maxStepCount,
	STEP_MAX != #inf.

nodeMeta(node(k,SYSTEM,NODE,ENTITY,ENTITY2), 0, 0) :- 
	not connectToNode(node(k,SYSTEM,NODE,ENTITY,ENTITY2), _),
	nodeCount(node(k,SYSTEM,NODE,ENTITY,ENTITY2),0,MAX).

% create output of nodes
stream(k,SYSTEM,node(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY,SCOPE,REP,ENTITY2),STEP_MAX+1) :- 
	node(k,SYSTEM,NODE,ENTITY,ENTITY2), 
	nodeMeta(node(k,SYSTEM,NODE,ENTITY,ENTITY2),STEP_MAX,_),
	ENTITY2 != none,
	ENTITY2 != any,
	output(SYSTEM,NODE,SCOPE,REP,ENTITY2).

stream(k,SYSTEM,node(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY,SCOPE,REP,none),STEP_MAX+1) :- 
	node(k,SYSTEM,NODE,ENTITY,ENTITY2), 
	nodeMeta(node(k,SYSTEM,NODE,ENTITY,ENTITY2),STEP_MAX,_),
	output(SYSTEM,NODE,SCOPE,REP,none).

stream(k,SYSTEM,node(k,SYSTEM,NODE,ENTITY,ENTITY2),information(ENTITY,SCOPE,REP,ENTITY2),STEP_MAX+1) :- 
	node(k,SYSTEM,NODE,ENTITY,ENTITY2), 
	nodeMeta(node(k,SYSTEM,NODE,ENTITY,ENTITY2),STEP_MAX,_),
	output(SYSTEM,NODE,SCOPE,REP,any).

% Speed up ASP
:- stream(k,SYSTEM,NODE,INFO,STEP2), 
   stream(k,SYSTEM,NODE,INFO,STEP), 
   STEP != STEP2.


%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% map guessing
%%% 
%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% creating a map
tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,none) :- 
	mapNodeTemplate(SYSTEM,NODE,ENTITY_TYPE),
	ENTITY_TYPE != any,
	query(k).

tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2) :- 
	mapNodeTemplate(SYSTEM,NODE,any),
	entity(ENTITY,ENTITY_TYPE), 
	hasScope(ENTITY_TYPE,SCOPE), 
	input(SYSTEM,NODE,SCOPE,REP,ENTITY2,_,_),
	query(k).

tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2) :- 
	mapNodeTemplate(SYSTEM,NODE,any),
	inputMap(SYSTEM,NODE,ENTITY_TYPE,_,_,ENTITY2,_,_),
	query(k).

% identify special nodes
snode(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2)) :-
	tmap(k,SYSTEM,tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE),
	tmapInputMap(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE2,_,_).
nnode(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2)) :-
	tmap(k,SYSTEM,tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE),
	tmapInputMap(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE2,_,_),
	INFO_TYPE != INFO_TYPE2.

nnode(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2)) :-
	tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),
	not tmapInputMap(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),_,_,_).

% computing theoretical maps
tmap(k,SYSTEM,tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2)) :- 
	tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	outputMap(SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2).


% computing theoretical inputs for maps
tinput(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),information(ENTITY,SCOPE,REP,ENTITY2),MIN,MAX) :-
	entity(ENTITY,ENTITY_TYPE),
	tmapInput(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENT2),MIN,MAX).

tmapInput(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),MIN,MAX) :- 
	tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	input(SYSTEM,NODE,SCOPE,REP,ENTITY2,MIN,MAX), 
	ENTITY2 != none, 
	ENTITY2 != any.

tmapInput(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,none),MIN,MAX) :- 
	tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	input(SYSTEM,NODE,SCOPE,REP,none,MIN,MAX).

tmapInput(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),MIN,MAX) :- 
	tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	input(SYSTEM,NODE,SCOPE,REP,any,MIN,MAX).


tmapInput(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),information(ENTITY2,SCOPE,REP,ENTITY),MIN,MAX) :- 
	tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),
	input2(SYSTEM,NODE,SCOPE,REP,ENTITY,MIN,MAX),
	ENTITY2 != none,
	ENTITY2 != any.

tmapInput(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),information(ENTITY2,SCOPE,REP,none),MIN,MAX) :- 
	tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	input2(SYSTEM,NODE,SCOPE,REP,none,MIN,MAX).

tmapInput(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),information(ENTITY2,SCOPE,REP,none),MIN,MAX) :- 
	tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	input2(SYSTEM,NODE,SCOPE,REP,any,MIN,MAX).


tmapInputMap(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),MIN,MAX) :- 
	tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	inputMap(SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2,MIN,MAX).


% inputs
connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),stream(k,SYSTEM,node(k,SOURCE,PROVIDER,ENTITY3,ENTITY4),INFO,STEP)) :-
	before(tnode(k,SOURCE,PROVIDER,ENTITY3,ENTITY4),tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO,S), 
	STEP < maxStepCount,
	PROVIDER != NODE,
	stream(k,SYSTEM,node(k,SOURCE,PROVIDER,ENTITY3,ENTITY4),INFO,STEP). 

connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),map(k,SYSTEM,mapNode(k,SOURCE,PROVIDER,ENTITY_TYPE2,ENTITY4),INFO_TYPE,STEP)) :-
	before(tmapNode(k,SOURCE,PROVIDER,ENTITY_TYPE2,ENTITY4),tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,S), 
	STEP < maxStepCount,
	PROVIDER != NODE,
	map(k,SYSTEM,mapNode(k,SOURCE,PROVIDER,ENTITY_TYPE2,ENTITY4),INFO_TYPE,STEP).


% outputs
mapCount(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 0, 100) :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2).

-maxStep(NODE,STEP) :-
	connectToMap(NODE,stream(k,_,_,_,STEP)),
	connectToMap(NODE,stream(k,_,_,_,STEP2)),
	STEP2 > STEP.
-maxStep(NODE,STEP) :-
	connectToMap(NODE,map(k,_,_,_,STEP)),
	connectToMap(NODE,map(k,_,_,_,STEP2)),
	STEP2 > STEP.
-maxStep(NODE,STEP) :-
	connectToMap(NODE,stream(k,_,_,_,STEP)),
	connectToMap(NODE,map(k,_,_,_,STEP2)),
	STEP2 > STEP.
-maxStep(NODE,STEP) :-
	connectToMap(NODE,map(k,_,_,_,STEP)),
	connectToMap(NODE,stream(k,_,_,_,STEP2)),
	STEP2 > STEP.

maxStep(NODE,STEP) :-
	connectToMap(NODE,stream(k,_,_,_,STEP)),
	not -maxStep(NODE,STEP).

maxStep(NODE,STEP) :-
	connectToMap(NODE,map(k,_,_,_,STEP)),
	not -maxStep(NODE,STEP).

mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), STEP_MAX, COUNT) :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),
	maxStep(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP_MAX),
	COUNT = #count{CONNECTED : connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),CONNECTED)}.

mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 0, 0) :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),
	not connectToMap(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),_).


% computing map output
map(k,SYSTEM,mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),STEP_MAX+1) :- 
	mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	outputMap(SYSTEM,NODE,ENTITY_TYPE,SCOPE,REP,ENTITY2),
	mapMeta(mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), STEP_MAX, COUNT).

% Speed up ASP
:- map(k,SYSTEM,MAP_NODE,INFO_TYPE,STEP2), 
   map(k,SYSTEM,MAP_NODE,INFO_TYPE,STEP), 
   STEP != STEP2.


%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% transfer between systems
%%% 
%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Information that could be used
requiredInformation(k,SYSTEM,INFO) :- 
	requiredStream(SYSTEM,INFO).

requiredInformation(k,SYSTEM,INFO) :- 
	tinput(tnode(k,SYSTEM,_,_,_),INFO,_,_).

requiredInformation(k,SYSTEM,information(ENTITY,SCOPE,REP,ENTITY2)) :- 
	tmapInput(tmapNode(k,SYSTEM,_,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),_,_),
	entity(ENTITY,ENTITY_TYPE).

requiredInformationType(k,SYSTEM,INFO_TYPE) :- 
	tmapInputMap(tmapNode(k,SYSTEM,_,_,_),INFO_TYPE,_,_).


% transfer a stream between systems
tstreamTransfer(SYSTEM_TARGET,tstream(k,SYSTEM_SOURCE,TNODE,INFO)) :-
	requiredInformation(k,SYSTEM_TARGET,INFO),
	transfer(SYSTEM_SOURCE,SYSTEM_TARGET),
	tstream(k,SYSTEM_SOURCE,TNODE,INFO),
	SYSTEM_TARGET != SYSTEM_SOURCE.

tstream(k,SYSTEM_TARGET,TNODE,INFO) :- 
	tstreamTransfer(SYSTEM_TARGET,tstream(k,SYSTEM,TNODE,INFO)).
	
stream(k,SYSTEM_TARGET,PROVIDER,INFO,STEP+1) :- 
	streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,PROVIDER,INFO,STEP),_).

:- streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE1,PROVIDER,INFO,_),_), 
   streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE2,PROVIDER,INFO,_),_), 
   SYSTEM_SOURCE1 != SYSTEM_SOURCE2.


% transfer a map between systems
tmapTransfer(SYSTEM_TARGET,tmap(k,SYSTEM_SOURCE,TNODE,INFO_TYPE)) :-
	requiredInformationType(k,SYSTEM_TARGET,INFO_TYPE),
	transfer(SYSTEM_SOURCE,SYSTEM_TARGET),
	tmap(k,SYSTEM_SOURCE,TNODE,INFO_TYPE),
	SYSTEM_TARGET != SYSTEM_SOURCE.

tmap(k,SYSTEM_TARGET,TMAP_NODE,INFO_TYPE) :- 
	tmapTransfer(SYSTEM_TARGET,tmap(k,SYSTEM,TMAP_NODE,INFO_TYPE)).
	
map(k,SYSTEM_TARGET,PROVIDER,INFO_TYPE,STEP+1) :- 
	mapTransfer(k,SYSTEM_TARGET,map(k,SYSTEM_SOURCE,PROVIDER,INFO_TYPE,STEP),_).

:- mapTransfer(k,SYSTEM_TARGET,map(k,SYSTEM_SOURCE1,PROVIDER,INFO_TYPE,_),_), 
   mapTransfer(k,SYSTEM_TARGET,map(k,SYSTEM_SOURCE2,PROVIDER,INFO_TYPE,_),_), 
   SYSTEM_SOURCE1 != SYSTEM_SOURCE2.


% transfer of information between islands
transfer(SYSTEM_SOURCE,SYSTEM_TARGET) :- 
	bridge(ISLAND1,ISLAND2),
	connectToBridge(SYSTEM_SOURCE,ISLAND2),
	connectToBridge(SYSTEM_TARGET,ISLAND1),
	system(SYSTEM_SOURCE,ISLAND1), 
	system(SYSTEM_TARGET,ISLAND2).

metadataProcessing(METADATA,SYSTEM_SOURCE,SYSTEM_TARGET,VALUE1+VALUE2+VALUE3) :- 
	bridge(ISLAND1,ISLAND2), 
	system(SYSTEM_SOURCE,ISLAND1), 
	system(SYSTEM_TARGET,ISLAND2),
	connectToBridge(SYSTEM_SOURCE,ISLAND2),
	connectToBridge(SYSTEM_TARGET,ISLAND1),
	metadataProcessing(METADATA,SYSTEM_SOURCE,ISLAND2,VALUE1),
	metadataProcessing(METADATA,SYSTEM_TARGET,ISLAND1,VALUE2),
	metadataProcessing(METADATA,ISLAND1,ISLAND2,VALUE3).

metadataOutput(METADATA,SYSTEM_SOURCE,SYSTEM_TARGET,VALUE1+VALUE2+VALUE3) :- 
	bridge(ISLAND1,ISLAND2), 
	system(SYSTEM_SOURCE,ISLAND1), 
	system(SYSTEM_TARGET,ISLAND2),
	connectToBridge(SYSTEM_SOURCE,ISLAND2),
	connectToBridge(SYSTEM_TARGET,ISLAND1),
	metadataOutput(METADATA,SYSTEM_SOURCE,ISLAND2,VALUE1),
	metadataOutput(METADATA,SYSTEM_TARGET,ISLAND1,VALUE2),
	metadataOutput(METADATA,ISLAND1,ISLAND2,VALUE3).


%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Checking
%%% soft optimization constraint to enable partial solutions
%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% streams
selectedStream(k,SYSTEM,node(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO,STEP) :-
	before(tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),tnode(k,SYSTEM,none,none,none),INFO,0),
	stream(k,SYSTEM,node(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO,STEP).
:~ requiredStream(SYSTEM,INFO), not selectedStream(k,SYSTEM,_,INFO,_), query(k). [1@5,requiredStream]

% maps
selectedMap(k,SYSTEM,mapNode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO_TYPE,STEP) :-
	before(tmapNode(k,SYSTEM2,NODE,ENTITY,ENTITY2),tnode(k,SYSTEM,none,none,none),INFO,0),
	map(k,SYSTEM,mapNode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO_TYPE,STEP).
:~ requiredMap(SYSTEM,INFO_TYPE), not selectedMap(k,SYSTEM,_,INFO_TYPE,_), query(k). [1@5,requiredMap]

% checking point budget of systems
reqPoints(k,SYSTEM,(SUM_NODES + SUM_STREAMS + SUM_MAP_NODES)) :- 
	system(SYSTEM,ISLAND), 
	SUM_STREAMS = #sum{COST,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,STEP : 
				streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),_), 
				metadataProcessing(cost,SYSTEM,SYSTEM_TARGET,COST)}, 
	SUM_NODES = #sum{COST,NODE,ENTITY,ENTITY2 : 
				node(k,SYSTEM,NODE,ENTITY,ENTITY2), 
				metadataProcessing(cost,SYSTEM,NODE,COST)}, 
	SUM_MAP_NODES = #sum{COST,NODE,ENTITY,ENTITY2 : 
				mapNode(k,SYSTEM,NODE,ENTITY,ENTITY2), 
				metadataProcessing(cost,SYSTEM,NODE,COST)},
	query(k).

:- reqPoints(k,SYSTEM,REQ_POINTS), systemPoints(SYSTEM,POINTS), REQ_POINTS > POINTS.

% transfer cost
%#minimize { COST@priorityTransferCost,SYSTEM,SYSTEM_TARGET,PROVIDER,INFO : 
%		streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM,PROVIDER,INFO,_),_), 
%		metadataProcessing(cost,SYSTEM,SYSTEM_TARGET,COST), 
%		query(k) }.

% cost
#minimize { COST@priorityCost,SYSTEM : reqPoints(k,SYSTEM,COST) }.



%--------------------------------------------------------------------------------------
#program entity(entity,entityType).

entity(entity,entityType).


%--------------------------------------------------------------------------------------
#program sourceNode(node,system,source,entity,scope,rep,entity2,delayFix,accuracyFix,costValue).

#external sourceNode(system,node,entity).

output(system,node,scope,rep,none).

metadataOutput(accuracy,system,node,fix,accuracyFix,0).
metadataOutput(delay,system,node,fix,delayFix,0).
metadataProcessing(cost,system,node,costValue).


%--------------------------------------------------------------------------------------
#program requiredStream(system,information,delayValue,accuracyValue).

#external requiredStream(system,information).
requiredMetadata(accuracy,system,information,accuracyValue).
requiredMetadata(delay,system,information,delayValue).


%--------------------------------------------------------------------------------------
#program requiredMap(system,informationType).

#external requiredMap(system,informationType).


%--------------------------------------------------------------------------------------
#program transfer(system1,system2,delayValue,costValue).

#external transfer(system1,system2).
transfer(system2,system1) :- transfer(system1,system2).

metadataProcessing(cost,system1,system2,costValue).
metadataOutput(delay,system1,system2,delayValue).

metadataProcessing(cost,system2,system1,costValue).
metadataOutput(delay,system2,system1,delayValue).


%--------------------------------------------------------------------------------------
#program system(name).

#external system(name,default).

%--------------------------------------------------------------------------------------
#program system(name,points).

#external system(name,default).
systemPoints(name,points) :- system(name,default).


%--------------------------------------------------------------------------------------
#program island(name).

#external island(name).


%--------------------------------------------------------------------------------------
#program system(name,points,island).

#external system(name,island).
systemPoints(name,points) :- system(name,island).


%--------------------------------------------------------------------------------------
#program connectToBridge(system,island,delayValue,costValue).

connectToBridge(system,island).
metadataProcessing(cost,system,island,costValue).
metadataOutput(delay,system,island,delayValue).


%--------------------------------------------------------------------------------------
#program bridge(island1,island2,delayValue,costValue).

#external bridge(island1,island2).
bridge(island2,island1) :- bridge(island1,island2).

metadataProcessing(cost,island1,island2,costValue).
metadataOutput(delay,island1,island2,delayValue).

metadataProcessing(cost,island2,island1,costValue).
metadataOutput(delay,island2,island1,delayValue).







