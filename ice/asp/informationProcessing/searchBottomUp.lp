%%%%--------------------------------------------------------------------------------------
%%%%
%%%% Bottom up search 
%%%% 
%%%% Implements a simple local search based on penalties for not optimal streams
%%%%
%%%%--------------------------------------------------------------------------------------
#program query(k,maxHopCount,maxStepCount).

% select a theoretical stream for each required stream
0 { before(tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),tnode(k,SYSTEM,none,none,none),INFO,0) : tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO) } 1 :- 
	requiredStream(SYSTEM,INFO), 
	query(k).

% select a theoretical map for each required map
0 { before(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),tnode(k,SYSTEM,none,none,none),SCOPE,REP,0) : tmap(k,SYSTEM,tnode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),ENTITY_TYPE,SCOPE,REP,ENTITY2) } 1 :- 
	requiredMap(SYSTEM,ENTITY_TYPE,SCOPE,REP,ENTITY2), 
	query(k).


% flagging nodes during search, identifies that a node will be before another node in the resulting processing graph
MIN { before(TNODE,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,S) : 
		tstream(k,SYSTEM,TNODE,INFO), 
		not before(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),TNODE,_),
		TNODE != tnode(k,SYSTEM,NODE,ENTITY,ENTITY2)} MAX :- 
	flagged(k,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),S), 
	S < maxStepCount, 
	tinput(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,MIN,MAX), 
	query(k).

MIN { before(TNODE,tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),information(ENTITY,SCOPE,REP,ENTITY2),S) : 
		tstream(k,SYSTEM,TNODE,information(ENTITY,SCOPE,REP,ENTITY2)), 
		entity(ENTITY,ENTITY_TYPE),
		not before(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),TNODE,_) } MAX :- 
	flagged(k,tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),S), 
	S < maxStepCount, 
	tmapInput(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),SCOPE,REP,MIN,MAX), 
	query(k).


% flag node as to used
flagged(k,TNODE,S+1) :- before(TNODE,_,_,S), query(k).
flagged(k,TMAPNODE,S+1) :- before(TMAPNODE,_,_,_,S), query(k).


% each flagged node will be instanced
node(k,SYSTEM,NODE,ENTITY,ENTITY2) :- 
	flagged(k,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),S),
	query(k),
	stream(k,SYSTEM,_,_,INFO,_) : tinput(tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,MIN,MAX).

mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2) :- 
	flagged(k,tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),S),
	query(k).%,
	%stream(k,SYSTEM,_,_,information(ENTITY,SCOPE,REP,ENTITY2),_) : tmapInput(tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),SCOPE,REP,MIN,MAX), entity(ENTITY,ENTITY_TYPE).


% transfer streams if nodes are from different systems
streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,NODE2,SYSTEM_SOURCE,INFO,STEP),TRANSFER_DELAY,TRANSFER_COST,0) :-
	before(tnode(k,SYSTEM_SOURCE,NODE2,ENTITY3,ENTITY4),tnode(k,SYSTEM_TARGET,NODE,ENTITY,ENTITY2),INFO,S),
	stream(k,SYSTEM_SOURCE,NODE2,SYSTEM_SOURCE,INFO,STEP),
	transfer(SYSTEM_SOURCE,SYSTEM_TARGET,TRANSFER_DELAY,TRANSFER_COST).

streamTransfer(k,SYSTEM_TARGET,stream(k,SYSTEM_SOURCE,NODE2,SYSTEM_SOURCE,INFO,STEP),TRANSFER_DELAY,TRANSFER_COST,0) :-
	before(tnode(k,SYSTEM_SOURCE,NODE2,ENTITY3,ENTITY4),tmapNode(k,SYSTEM_TARGET,NODE,ENTITY_TYPE,ENTITY2),INFO,S),
	stream(k,SYSTEM_SOURCE,NODE2,SYSTEM_SOURCE,INFO,STEP),
	transfer(SYSTEM_SOURCE,SYSTEM_TARGET,TRANSFER_DELAY,TRANSFER_COST).


% flagged nodes which are not instanced are forbidden
:- flagged(k,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),S), 
   not node(k,SYSTEM,NODE,ENTITY,ENTITY2).

:- flagged(k,tmapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),S), 
   not mapNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2).


%:- before(tnode(k,SYSTEM,NODE2,ENTITY,ENTITY2), tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),_,S), iro(SYSTEM,NODE,_,_), iro(SYSTEM,NODE2,_,_).
%*
iroChain(ENTITY,SCOPE,ENTITY2,REP1,REP3) :- iroChain(ENTITY,SCOPE,ENTITY2,REP1,REP2), iroChain(ENTITY,SCOPE,ENTITY2,REP2,REP3). 

iroChain(ENTITY,SCOPE,ENTITY2,REP1,REP2) :-
	before(tnode(k,SYSTEM,NODE2,ENTITY,ENTITY2), tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),_,_),
	iro(SYSTEM,NODE,_,_),
	stream(k,SYSTEM,_,_,information(ENTITY,SCOPE,REP2,ENTITY2),STEP+1),
	stream(k,SYSTEM,_,_,information(ENTITY,SCOPE,REP1,ENTITY2),STEP),
	REP2 != REP1.

:- iroChain(ENTITY,SCOPE,ENTITY2,REP,REP,S).
*%
