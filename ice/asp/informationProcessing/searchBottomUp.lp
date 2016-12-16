%%%%--------------------------------------------------------------------------------------
%%%%
%%%% Bottom up search 
%%%% 
%%%% Implements a simple local search based on penalties for not optimal streams
%%%%
%%%%--------------------------------------------------------------------------------------
#program query(k,maxHopCount,maxStepCount).

% select a theoretical stream for each required stream
0 { before(TNODE,tnode(k,SYSTEM,none,none,none),INFO,0) : tstream(k,SYSTEM,TNODE,INFO) } 1 :- 
	requiredStream(SYSTEM,INFO), 
	query(k).

% select a theoretical set for each required set
0 { before(TNODE,tnode(k,SYSTEM,none,none,none),INFO_TYPE,0) : tset(k,SYSTEM,TNODE,INFO_TYPE) } 1 :- 
	requiredSet(SYSTEM,INFO_TYPE), 
	query(k).


% flagging nodes during search, identifies that a node will be before another node in the resulting processing graph
%% stream concatination
MIN { before(TNODE_BEFORE,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,S) : 
		tstream(k,SYSTEM,TNODE_BEFORE,INFO), 
		nnode(TNODE_BEFORE)
		} MAX :- 
	flagged(k,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),S), 
	snode(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2)), 
	S < maxStepCount, 
	tinput(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,MIN,MAX), 
	query(k).

MIN { before(TNODE_BEFORE,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,S) : 
		tstream(k,SYSTEM,TNODE_BEFORE,INFO)
		} MAX :-  
	flagged(k,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),S), 
	S < maxStepCount, 
	nnode(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2)),
	tinput(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,MIN,MAX), 
	query(k).


%% set concatination
MIN { before(TNODE_BEFORE,tsetNode(k,SYSTEM,NODE,ENTITY3,ENTITY4),information(ENTITY,SCOPE,REP,ENTITY2),S) : 
		tstream(k,SYSTEM,TNODE_BEFORE,information(ENTITY,SCOPE,REP,ENTITY2)),
		entity(ENTITY,ENTITY_TYPE)
		} MAX :- 
	flagged(k,tsetNode(k,SYSTEM,NODE,ENTITY3,ENTITY4),S), 
	S < maxStepCount, 
	nnode(tsetNode(k,SYSTEM,NODE,ENTITY3,ENTITY4)),
	tsetInput(tsetNode(k,SYSTEM,NODE,ENTITY3,ENTITY4),informationType(ENTITY_TYPE,SCOPE,REP,ENTITY2),MIN,MAX),
	query(k).

MIN { before(TNODE,tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,S) : 
		tset(k,SYSTEM,TNODE,INFO_TYPE),
		before(tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),TNODE2,_,S-1),
		TNODE != TNODE2
		} MAX :- 
	flagged(k,tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),S), 
	nnode(tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2)),
	S < maxStepCount, 
	tsetInputSet(tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,MIN,MAX), 
	query(k).

MIN { before(TNODE,tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,S) : 
		tset(k,SYSTEM,TNODE,INFO_TYPE),
		before(tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),TNODE2,_,S-1),
		TNODE != TNODE2,
		nnode(TNODE)
		} MAX :- 
	flagged(k,tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),S), 
	snode(tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2)),
	S < maxStepCount, 
	tsetInputSet(tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,MIN,MAX), 
	query(k).

% flag node as to used
flagged(k,TNODE,S+1) :- before(TNODE,_,_,S), query(k).


% each flagged node will be instanced
node(k,SYSTEM,NODE,ENTITY,ENTITY2) :- 
	flagged(k,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),S),
	query(k).
setNode(k,SYSTEM,NODE,ENTITY_TYPE,none) :-
	flagged(k,tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,none),S),
	query(k).

% transfer streams if nodes are from different systems
streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,node(k,SYSTEM_SOURCE,NODE2,ENTITY3,ENTITY4),INFO,STEP),0) :-
	before(tnode(k,SYSTEM_SOURCE,NODE2,ENTITY3,ENTITY4),tnode(k,SYSTEM_TARGET,NODE,ENTITY,ENTITY2),INFO,S),
	stream(k,SYSTEM_SOURCE,node(k,SYSTEM_SOURCE,NODE2,ENTITY3,ENTITY4),INFO,STEP),
	transfer(SYSTEM_SOURCE,SYSTEM_TARGET).

streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,node(k,SYSTEM_SOURCE,NODE2,ENTITY3,ENTITY4),INFO,STEP),0) :-
	before(tnode(k,SYSTEM_SOURCE,NODE2,ENTITY3,ENTITY4),tsetNode(k,SYSTEM_TARGET,NODE,ENTITY_TYPE,ENTITY2),INFO,S),
	stream(k,SYSTEM_SOURCE,node(k,SYSTEM_SOURCE,NODE2,ENTITY3,ENTITY4),INFO,STEP),
	transfer(SYSTEM_SOURCE,SYSTEM_TARGET).

setTransfer(k,SYSTEM_TARGET,set(k,SYSTEM_SOURCE,setNode(k,SYSTEM_SOURCE,NODE2,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP),0) :-
	before(tsetNode(k,SYSTEM_SOURCE,NODE2,ENTITY_TYPE,ENTITY2),tsetNode(k,SYSTEM_TARGET,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,S),
	set(k,SYSTEM_SOURCE,setNode(k,SYSTEM_SOURCE,NODE2,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP),
	transfer(SYSTEM_SOURCE,SYSTEM_TARGET).


% flagged nodes which are not instanced are forbidden
:- flagged(k,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),S), 
   not node(k,SYSTEM,NODE,ENTITY,ENTITY2).

:- flagged(k,tsetNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),S), 
   not setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2).



