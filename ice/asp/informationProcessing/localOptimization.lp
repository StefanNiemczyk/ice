%%%%--------------------------------------------------------------------------------------
%%%%
%%%% Local optimization 
%%%% 
%%%% Implements a simple local search based on penalties for not optimal streams
%%%%
%%%%--------------------------------------------------------------------------------------
#program query(k,maxHopCount,maxStepCount).

% heuristic of node output QoS
heuristic(k,tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO),METADATA,1000-SUM) :-
	metadata(METADATA,minimize),
	SYSTEM = SYSTEM2,
	tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO),
	metadataNode(METADATA,SYSTEM2,NODE,TYPE,FIX_VALUE,MOD_VALUE),
	nodeCount(k,SYSTEM2,NODE,MIN,MAX),
	SUM = FIX_VALUE+(MAX-MIN+1)*MOD_VALUE.

heuristic(k,tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO),METADATA,1000-SUM) :-
	metadata(METADATA,minimize),
	METADATA != delay,
	SYSTEM != SYSTEM2,
	tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO),
	metadataNode(METADATA,SYSTEM2,NODE,TYPE,FIX_VALUE,MOD_VALUE),
	nodeCount(k,SYSTEM2,NODE,MIN,MAX),
	SUM = FIX_VALUE+(MAX-MIN+1)*MOD_VALUE.

heuristic(k,tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO),delay,1000-SUM) :-
	metadata(delay,minimize),
	SYSTEM != SYSTEM2,
	tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO),
	metadataNode(delay,SYSTEM2,NODE,TYPE,FIX_VALUE,MOD_VALUE),
	nodeCount(k,SYSTEM2,NODE,MIN,MAX),
	transfer(SYSTEM2,SYSTEM,TRANSFER_DELAY,_),
	SUM = FIX_VALUE+(MAX-MIN+1)*MOD_VALUE + TRANSFER_DELAY.

heuristic(k,tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO),METADATA,SUM) :-
	metadata(METADATA,maximize),
	tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO),
	metadataNode(METADATA,SYSTEM2,NODE,TYPE,FIX_VALUE,MOD_VALUE),
	nodeCount(k,SYSTEM2,NODE,MIN,MAX),
	SUM = FIX_VALUE+(MAX-MIN+1)*MOD_VALUE.


% see raveling salespersons problem for details
%order(X,C1,C2) :- outcost(X,(C1;C2)), C1 < C2, C <= C1 : outcost(X,C), C < C2.
order(k,SYSTEM,METADATA,INFO,C1,C2) :- 
	metadata(METADATA,_),
	heuristic(k,tstream(k,SYSTEM,_,INFO),METADATA,(C1;C2)),
	C1 < C2, 
	C <= C1 : heuristic(k,tstream(k,SYSTEM,_,INFO),METADATA,C), C < C2.

%penalty(X,C1,C2-C1) :- order(X,C1,C2), cycle(X,Y), cost(X,Y,C2).
penalty(k,SYSTEM,METADATA,INFO,C2,C2-C1) :-
	metadata(METADATA,_), 
	order(k,SYSTEM,METADATA,INFO,C1,C2), 
	flagged(k,TNODE,_), tstream(k,SYSTEM,TNODE,INFO),
	heuristic(k,tstream(k,SYSTEM,TNODE,INFO),METADATA,C1).

%penalty(X,C1,C2-C1) :- order(X,C1,C2), penalty(X,C2,_).
penalty(k,SYSTEM,METADATA,INFO,C2,C2-C1) :- 
	order(k,SYSTEM,METADATA,INFO,C1,C2), 
	penalty(k,SYSTEM,METADATA,INFO,C1,_).

% TODO add configurable settings for different metadata
#minimize { D@3,SYSTEM,METADATA,INFO,C : penalty(k,SYSTEM,METADATA,INFO,C,D) }.









