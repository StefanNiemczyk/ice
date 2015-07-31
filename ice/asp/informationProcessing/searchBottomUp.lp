%%%%--------------------------------------------------------------------------------------
%%%%
%%%% Bottom up search 
%%%% 
%%%% Implements a simple local search based on penalties for not optimal streams
%%%%
%%%%--------------------------------------------------------------------------------------
#program query(k,maxHopCount,maxStepCount).

% select a theoretical stream for each required stream
1 { before(tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),tnode(k,SYSTEM,none,none,none),INFO,0) : tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO) } 1 :- 
	requiredStream(SYSTEM,INFO), 
	query(k).

% select a theoretical map for each required map
0 { before(tmapNode(k,SYSTEM2,NODE,ENTITY_TYPE,ENTITY2),tnode(k,SYSTEM,none,none,none),INFO_TYPE,0) : tmap(k,SYSTEM,tmapNode(k,SYSTEM2,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE) } 1 :- 
	requiredMap(SYSTEM,INFO_TYPE), 
	query(k).


% flagging nodes during search, identifies that a node will be before another node in the resulting processing graph
MIN { before(TNODE_BEFORE,TNODE,INFO,S) : 
		tstream(k,SYSTEM,TNODE_BEFORE,INFO), 
		not before(TNODE,TNODE_BEFORE,_,_), nnode(TNODE_BEFORE)} MAX :- 
	flagged(k,TNODE,S), 
	snode(TNODE), 
	S < maxStepCount, 
	tinput(TNODE,INFO,MIN,MAX), 
	query(k).

MIN { before(TNODE_BEFORE,TNODE,INFO,S) : 
		tstream(k,SYSTEM,TNODE_BEFORE,INFO), 
		not before(TNODE,TNODE_BEFORE,_,_)} MAX :- 
	flagged(k,TNODE,S), 
	S < maxStepCount, 
	nnode(TNODE),
	tinput(TNODE,INFO,MIN,MAX), 
	query(k).

MIN { before(TNODE,tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,S) : 
		tmap(k,SYSTEM,TNODE,INFO_TYPE) } MAX :- 
	flagged(k,tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),S), 
	S < maxStepCount, 
	tmapInputMap(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,MIN,MAX), 
	query(k).


% flag node as to used
:- flagged(k,TNODE,S1), flagged(k,TNODE,S2), S1 != S2, query(k).
flagged(k,TNODE,S+1) :- before(TNODE,_,_,S), query(k).


% each flagged node will be instanced
node(k,SYSTEM,NODE,ENTITY,ENTITY2) :- 
	flagged(k,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),S),
	query(k).

mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2) :- 
	flagged(k,tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),S),
	query(k).


% transfer streams if nodes are from different systems
streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,node(k,SYSTEM_SOURCE,NODE2,ENTITY3,ENTITY4),INFO,STEP),0) :-
	before(tnode(k,SYSTEM_SOURCE,NODE2,ENTITY3,ENTITY4),tnode(k,SYSTEM_TARGET,NODE,ENTITY,ENTITY2),INFO,S),
	stream(k,SYSTEM_SOURCE,node(k,SYSTEM_SOURCE,NODE2,ENTITY3,ENTITY4),INFO,STEP),
	transfer(SYSTEM_SOURCE,SYSTEM_TARGET).

streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,node(k,SYSTEM_SOURCE,NODE2,ENTITY3,ENTITY4),INFO,STEP),0) :-
	before(tnode(k,SYSTEM_SOURCE,NODE2,ENTITY3,ENTITY4),tmapNode(k,SYSTEM_TARGET,NODE,ENTITY_TYPE,ENTITY2),INFO,S),
	stream(k,SYSTEM_SOURCE,node(k,SYSTEM_SOURCE,NODE2,ENTITY3,ENTITY4),INFO,STEP),
	transfer(SYSTEM_SOURCE,SYSTEM_TARGET).

mapTransfer(k,SYSTEM_TARGET,map(k,SYSTEM_SOURCE,mapNode(k,SYSTEM_SOURCE,NODE2,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP),0) :-
	before(tmapNode(k,SYSTEM_SOURCE,NODE2,ENTITY_TYPE,ENTITY2),tmapNode(k,SYSTEM_TARGET,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,S),
	map(k,SYSTEM_SOURCE,mapNode(k,SYSTEM_SOURCE,NODE2,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP),
	transfer(SYSTEM_SOURCE,SYSTEM_TARGET).


% flagged nodes which are not instanced are forbidden
:- flagged(k,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),S), 
   not node(k,SYSTEM,NODE,ENTITY,ENTITY2).

:- flagged(k,tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),S), 
   not mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2).



