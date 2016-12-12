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
metadata(set,density,maximize).
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
 
tnode(k,SYSTEM,NODE,ENTITY,ENTITY2) :- 
	transformation(SYSTEM,NODE,any,any), 
	entity(ENTITY,ENTITY_TYPE), 
	hasScope(ENTITY_TYPE,SCOPE), 
	input(SYSTEM,NODE,SCOPE,_,_,_,_),
	entity(ENTITY2,ENTITY_TYPE2), 
	hasScope(ENTITY_TYPE2,SCOPE2), 
	input2(SYSTEM,NODE,SCOPE2,_,_,_,_),
	query(k).
tnode(k,SYSTEM,NODE,ENTITY,none) :- 
	transformation(SYSTEM,NODE,any,none), 
	entity(ENTITY,ENTITY_TYPE), 
	hasScope(ENTITY_TYPE,SCOPE), 
	input(SYSTEM,NODE,SCOPE,_,_,_,_),
	query(k).

tnode(k,SYSTEM,NODE,ENTITY,ENTITY2) :- 
	autoTrans(SYSTEM,NODE,any,any), 
	entity(ENTITY,ENTITY_TYPE), 
	hasScope(ENTITY_TYPE,SCOPE), 
	input(SYSTEM,NODE,SCOPE,_,_,_,_),
	entity(ENTITY2,ENTITY_TYPE2), 
	hasScope(ENTITY_TYPE2,SCOPE2), 
	input2(SYSTEM,NODE,SCOPE2,_,_,_,_),
	query(k).
tnode(k,SYSTEM,NODE,ENTITY,none) :- 
	autoTrans(SYSTEM,NODE,any,none), 
	entity(ENTITY,ENTITY_TYPE), 
	hasScope(ENTITY_TYPE,SCOPE), 
	input(SYSTEM,NODE,SCOPE,_,_,_,_),
	query(k).

% identify special nodes
%snode(TNODE) :-
%	tstream(k,SYSTEM,TNODE,INFO),
%	tinput(TNODE,INFO,_,_).
%snode(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2)) :-
%	tnode(k,SYSTEM,IRO,ENTITY,ENTITY2),
%	transformation(SYSTEM,NODE,_,_).
%nnode(TNODE) :-
%	tstream(k,SYSTEM,TNODE,INFO),
%	tinput(TNODE,INFO2,_,_),
%	INFO != INFO2.
%nnode(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2)) :-
%	sourceNode(SYSTEM,NODE,_),
%	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2).

snode(tnode(k, SYSTEM, NODE, ENTITY, ENTITY2)) :-
	tstream(k, SYSTEM,tnode(k, SYSTEM, NODE, ENTITY, ENTITY2), INFO),
	tinput(tnode(k, SYSTEM, NODE, ENTITY, ENTITY2), INFO, _, _).
nnode(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2)) :-
	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),
	autoTrans(SYSTEM,NODE,_,_).
nnode(tnode(k, SYSTEM, NODE, ENTITY, ENTITY2)) :-
	nodeTemplate(SYSTEM,NODE,_),
	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),
	tstream(k, SYSTEM, tnode(k,SYSTEM,NODE,ENTITY,ENTITY2), INFO),
	tinput(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2), INFO2, _, _),
	INFO != INFO2.
nnode(tnode(k, SYSTEM, NODE, ENTITY, ENTITY2)) :-
	transformation(SYSTEM,NODE,_,_),
	tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),
	tstream(k, SYSTEM, tnode(k,SYSTEM,NODE,ENTITY,ENTITY2), INFO),
	tinput(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2), INFO2, _, _),
	INFO != INFO2.
nnode(tnode(k, SYSTEM, NODE, ENTITY, ENTITY2)) :-
	sourceNode(SYSTEM, NODE, _),
	tnode(k, SYSTEM, NODE, ENTITY, ENTITY2).
	

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

tnodeCount(k,SYSTEM,NODE,MINV+MINV2,MAXV+MAXV2) :- 
	transformation(SYSTEM,NODE,_,_), 
	MINV = #sum{MIN,SCOPE,REP,ENT2 : input(SYSTEM,NODE,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV = #sum{MAX,SCOPE,REP,ENT2 : input(SYSTEM,NODE,SCOPE,REP,ENT2,_,MAX)}, 
	MINV2 = #sum{MIN,SCOPE,REP,ENT2 : input2(SYSTEM,NODE,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV2 = #sum{MAX,SCOPE,REP,ENT2 : input2(SYSTEM,NODE,SCOPE,REP,ENT2,_,MAX)}.

tnodeCount(k,SYSTEM,NODE,MINV+MINV2,MAXV+MAXV2) :- 
	autoTrans(SYSTEM,NODE,_,_), 
	MINV = #sum{MIN,SCOPE,REP,ENT2 : input(SYSTEM,NODE,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV = #sum{MAX,SCOPE,REP,ENT2 : input(SYSTEM,NODE,SCOPE,REP,ENT2,_,MAX)}, 
	MINV2 = #sum{MIN,SCOPE,REP,ENT2 : input2(SYSTEM,NODE,SCOPE,REP,ENT2,MIN,_)}, 
	MAXV2 = #sum{MAX,SCOPE,REP,ENT2 : input2(SYSTEM,NODE,SCOPE,REP,ENT2,_,MAX)}.

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
%%% set guessing
%%% 
%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% creating a set
tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,none) :- 
	setNodeTemplate(SYSTEM,NODE,ENTITY_TYPE),
	ENTITY_TYPE != any,
	query(k).

tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2) :- 
	setNodeTemplate(SYSTEM,NODE,any),
	hasScope(ENTITY_TYPE,SCOPE), 
	input(SYSTEM,NODE,SCOPE,REP,ENTITY2,_,_),
	query(k).

tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2) :- 
	setNodeTemplate(SYSTEM,NODE,any),
	hasScope(ENTITY_TYPE,SCOPE), 
	inputSet(SYSTEM,NODE,any,SCOPE,_,ENTITY2,_,_),
	query(k).


tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2) :- 
	setNodeTemplate(SYSTEM,NODE,any),
	ENTITY_TYPE != any,
	inputSet(SYSTEM,NODE,ENTITY_TYPE,_,_,ENTITY2,_,_),
	query(k).

% identify special nodes
snode(TNODE) :-
	tset(k,SYSTEM,TNODE,INFO_TYPE),
	tsetInputSet(TNODE,INFO_TYPE,_,_).
nnode(TNODE) :-
	tset(k,SYSTEM,TNODE,INFO_TYPE),
	tsetInputSet(TNODE,INFO_TYPE2,_,_),
	INFO_TYPE != INFO_TYPE2.

nnode(tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2)) :-
	tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),
	not tsetInputSet(tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),_,_,_).

% computing theoretical sets
tset(k,SYSTEM,tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2)) :- 
	tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	outputSet(SYSTEM,NODE,_,SCOPE,REP,ENTITY2).


% computing theoretical inputs for sets
%tinput(tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),information(ENTITY,SCOPE,REP,ENTITY2),MIN,MAX) :-
%	entity(ENTITY,ENTITY_TYPE),
%	tsetInput(tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENT2),MIN,MAX).

tsetInput(tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),MIN,MAX) :- 
	tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	input(SYSTEM,NODE,SCOPE,REP,ENTITY2,MIN,MAX), 
	ENTITY2 != none, 
	ENTITY2 != any.

tsetInput(tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,none),MIN,MAX) :- 
	tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	input(SYSTEM,NODE,SCOPE,REP,none,MIN,MAX).

tsetInput(tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),MIN,MAX) :- 
	tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	input(SYSTEM,NODE,SCOPE,REP,any,MIN,MAX).


tsetInput(tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),information(ENTITY2,SCOPE,REP,ENTITY),MIN,MAX) :- 
	tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),
	input2(SYSTEM,NODE,SCOPE,REP,ENTITY,MIN,MAX),
	ENTITY2 != none,
	ENTITY2 != any.

tsetInput(tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),information(ENTITY2,SCOPE,REP,none),MIN,MAX) :- 
	tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	input2(SYSTEM,NODE,SCOPE,REP,none,MIN,MAX).

tsetInput(tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),information(ENTITY2,SCOPE,REP,none),MIN,MAX) :- 
	tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	input2(SYSTEM,NODE,SCOPE,REP,any,MIN,MAX).


tsetInputSet(tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),MIN,MAX) :- 
	tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	inputSet(SYSTEM,NODE,_,SCOPE,REP,ENTITY2,MIN,MAX).


% inputs
connectToSet(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),stream(k,SYSTEM,node(k,SOURCE,PROVIDER,ENTITY3,ENTITY4),INFO,STEP)) :-
	before(tnode(k,SOURCE,PROVIDER,ENTITY3,ENTITY4),tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO,_), 
	STEP < maxStepCount,
	PROVIDER != NODE,
	stream(k,SYSTEM,node(k,SOURCE,PROVIDER,ENTITY3,ENTITY4),INFO,STEP). 

connectToSet(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),set(k,SYSTEM,setNode(k,SOURCE,PROVIDER,ENTITY_TYPE2,ENTITY4),INFO_TYPE,STEP)) :-
	before(tsetNode(k,SOURCE,PROVIDER,ENTITY_TYPE2,ENTITY4),tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,_), 
	STEP < maxStepCount,
	PROVIDER != NODE,
	set(k,SYSTEM,setNode(k,SOURCE,PROVIDER,ENTITY_TYPE2,ENTITY4),INFO_TYPE,STEP).

-maxStep(NODE,STEP) :-
	connectToSet(NODE,stream(k,_,_,_,STEP)),
	connectToSet(NODE,stream(k,_,_,_,STEP2)),
	STEP2 > STEP.
-maxStep(NODE,STEP) :-
	connectToSet(NODE,set(k,_,_,_,STEP)),
	connectToSet(NODE,set(k,_,_,_,STEP2)),
	STEP2 > STEP.
-maxStep(NODE,STEP) :-
	connectToSet(NODE,stream(k,_,_,_,STEP)),
	connectToSet(NODE,set(k,_,_,_,STEP2)),
	STEP2 > STEP.
-maxStep(NODE,STEP) :-
	connectToSet(NODE,set(k,_,_,_,STEP)),
	connectToSet(NODE,stream(k,_,_,_,STEP2)),
	STEP2 > STEP.

maxStep(NODE,STEP) :-
	connectToSet(NODE,stream(k,_,_,_,STEP)),
	not -maxStep(NODE,STEP).

maxStep(NODE,STEP) :-
	connectToSet(NODE,set(k,_,_,_,STEP)),
	not -maxStep(NODE,STEP).

setMeta(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), STEP_MAX, COUNT) :- 
	setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),
	maxStep(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),STEP_MAX),
	COUNT = #count{CONNECTED : connectToSet(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),CONNECTED)}.

setMeta(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 0, 0) :- 
	setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),
	not connectToSet(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),_).


% computing set output
set(k,SYSTEM,setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),STEP_MAX+1) :- 
	setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), 
	outputSet(SYSTEM,NODE,_,SCOPE,REP,ENTITY2),
	setMeta(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), STEP_MAX, COUNT).

% Speed up ASP
:- set(k,SYSTEM,NODE,INFO_TYPE,STEP2), 
   set(k,SYSTEM,NODE,INFO_TYPE,STEP), 
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
	tsetInput(tsetNode(k,SYSTEM,_,ENTITY_TYPE,ENTITY2),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),_,_),
	entity(ENTITY,ENTITY_TYPE).

requiredInformationType(k,SYSTEM,INFO_TYPE) :- 
	tsetInputSet(tsetNode(k,SYSTEM,_,_,_),INFO_TYPE,_,_).


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


% transfer a set between systems
tsetTransfer(SYSTEM_TARGET,tset(k,SYSTEM_SOURCE,TNODE,INFO_TYPE)) :-
	requiredInformationType(k,SYSTEM_TARGET,INFO_TYPE),
	transfer(SYSTEM_SOURCE,SYSTEM_TARGET),
	tset(k,SYSTEM_SOURCE,TNODE,INFO_TYPE),
	SYSTEM_TARGET != SYSTEM_SOURCE.

tset(k,SYSTEM_TARGET,TNODE,INFO_TYPE) :- 
	tsetTransfer(SYSTEM_TARGET,tset(k,SYSTEM,TNODE,INFO_TYPE)).
	
set(k,SYSTEM_TARGET,PROVIDER,INFO_TYPE,STEP+1) :- 
	setTransfer(k,SYSTEM_TARGET,set(k,SYSTEM_SOURCE,PROVIDER,INFO_TYPE,STEP),_).

:- setTransfer(k,SYSTEM_TARGET,set(k,SYSTEM_SOURCE1,PROVIDER,INFO_TYPE,_),_), 
   setTransfer(k,SYSTEM_TARGET,set(k,SYSTEM_SOURCE2,PROVIDER,INFO_TYPE,_),_), 
   SYSTEM_SOURCE1 != SYSTEM_SOURCE2.



%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Checking
%%% soft optimization constraint to enable partial solutions
%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% streams
selectedStream(k,SYSTEM,node(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO,STEP) :-
	before(tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),tnode(k,SYSTEM,none,none,none),INFO,0),
	stream(k,SYSTEM,node(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO,STEP).
:~ requiredStream(SYSTEM,INFO), not selectedStream(k,SYSTEM,_,INFO,_), query(k). [1@5,requiredStream]

% sets
selectedSet(k,SYSTEM,setNode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO_TYPE,STEP) :-
	before(tsetNode(k,SYSTEM2,NODE,ENTITY,ENTITY2),tnode(k,SYSTEM,none,none,none),INFO,0),
	set(k,SYSTEM,setNode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO_TYPE,STEP).
:~ requiredSet(SYSTEM,INFO_TYPE), not selectedSet(k,SYSTEM,_,INFO_TYPE,_), query(k). [1@5,requiredSet]

% checking point budget of systems
reqPoints(k,SYSTEM,(SUM_NODES + SUM_STREAMS + SUM_SET_NODES)) :- 
	system(SYSTEM), 
	SUM_STREAMS = #sum{COST,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,STEP : 
				streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP),_), 
				metadataProcessing(cost,SYSTEM,SYSTEM_TARGET,COST)}, 
	SUM_NODES = #sum{COST,NODE,ENTITY,ENTITY2 : 
				node(k,SYSTEM,NODE,ENTITY,ENTITY2), 
				metadataProcessing(cost,SYSTEM,NODE,COST)}, 
	SUM_SET_NODES = #sum{COST,NODE,ENTITY,ENTITY2 : 
				setNode(k,SYSTEM,NODE,ENTITY,ENTITY2), 
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
#program requiredSet(system,informationType).

#external requiredSet(system,informationType).


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

#external system(name).

%--------------------------------------------------------------------------------------
#program system(name,points).

#external system(name).
systemPoints(name,points) :- system(name).









