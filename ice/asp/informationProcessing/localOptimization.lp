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
	metadata(information,METADATA,minimize),
	SYSTEM = SYSTEM2,
	tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO),
	metadataOutput(METADATA,SYSTEM2,NODE,TYPE,FIX_VALUE,MOD_VALUE),
	nodeCount(k,SYSTEM2,NODE,MIN,MAX),
	SUM = FIX_VALUE+(MAX-MIN+1)*MOD_VALUE.

heuristic(k,tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO),METADATA,1000-SUM) :-
	metadata(stream,METADATA,minimize),
	SYSTEM = SYSTEM2,
	tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO),
	metadataOutput(METADATA,SYSTEM2,NODE,TYPE,FIX_VALUE,MOD_VALUE),
	nodeCount(k,SYSTEM2,NODE,MIN,MAX),
	SUM = FIX_VALUE+(MAX-MIN+1)*MOD_VALUE.


heuristic(k,tstream(k,SYSTEM_TARGET,tnode(k,SYSTEM_SOURCE,NODE,ENTITY,ENTITY2),INFO),METADATA,VALUE+VALUE_PROCESSING) :-
	metadata(information,METADATA,_),
	SYSTEM_TARGET != SYSTEM_SOURCE,
	tstream(k,SYSTEM_TARGET,tnode(k,SYSTEM_SOURCE,NODE,ENTITY,ENTITY2),INFO),
	heuristic(k,tstream(k,SYSTEM_SOURCE,tnode(k,SYSTEM_SOURCE,NODE,ENTITY,ENTITY2),INFO),METADATA,VALUE),
	metadataOutput(METADATA,SYSTEM_SOURCE,SYSTEM_TARGET,VALUE_PROCESSING).

heuristic(k,tstream(k,SYSTEM_TARGET,tnode(k,SYSTEM_SOURCE,NODE,ENTITY,ENTITY2),INFO),METADATA,VALUE) :-
	metadata(information,METADATA,_),
	SYSTEM_TARGET != SYSTEM_SOURCE,
	tstream(k,SYSTEM_TARGET,tnode(k,SYSTEM_SOURCE,NODE,ENTITY,ENTITY2),INFO),
	heuristic(k,tstream(k,SYSTEM_SOURCE,tnode(k,SYSTEM_SOURCE,NODE,ENTITY,ENTITY2),INFO),METADATA,VALUE),
	not metadataOutput(METADATA,SYSTEM_SOURCE,SYSTEM_TARGET,_).


heuristic(k,tstream(k,SYSTEM_TARGET,tnode(k,SYSTEM_SOURCE,NODE,ENTITY,ENTITY2),INFO),METADATA,VALUE+VALUE_PROCESSING) :-
	metadata(stream,METADATA,_),
	SYSTEM_TARGET != SYSTEM_SOURCE,
	tstream(k,SYSTEM_TARGET,tnode(k,SYSTEM_SOURCE,NODE,ENTITY,ENTITY2),INFO),
	heuristic(k,tstream(k,SYSTEM_SOURCE,tnode(k,SYSTEM_SOURCE,NODE,ENTITY,ENTITY2),INFO),METADATA,VALUE),
	metadataOutput(METADATA,SYSTEM_SOURCE,SYSTEM_TARGET,VALUE_PROCESSING).

heuristic(k,tstream(k,SYSTEM_TARGET,tnode(k,SYSTEM_SOURCE,NODE,ENTITY,ENTITY2),INFO),METADATA,VALUE) :-
	metadata(stream,METADATA,_),
	SYSTEM_TARGET != SYSTEM_SOURCE,
	tstream(k,SYSTEM_TARGET,tnode(k,SYSTEM_SOURCE,NODE,ENTITY,ENTITY2),INFO),
	heuristic(k,tstream(k,SYSTEM_SOURCE,tnode(k,SYSTEM_SOURCE,NODE,ENTITY,ENTITY2),INFO),METADATA,VALUE),
	not metadataOutput(METADATA,SYSTEM_SOURCE,SYSTEM_TARGET,_).




heuristic(k,tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO),METADATA,SUM) :-
	metadata(information,METADATA,maximize),
	SYSTEM = SYSTEM2,
	tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO),
	metadataOutput(METADATA,SYSTEM2,NODE,TYPE,FIX_VALUE,MOD_VALUE),
	nodeCount(k,SYSTEM2,NODE,MIN,MAX),
	SUM = FIX_VALUE+(MAX-MIN+1)*MOD_VALUE.

heuristic(k,tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO),METADATA,SUM) :-
	metadata(stream,METADATA,maximize),
	SYSTEM = SYSTEM2,
	tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO),
	metadataOutput(METADATA,SYSTEM2,NODE,TYPE,FIX_VALUE,MOD_VALUE),
	nodeCount(k,SYSTEM2,NODE,MIN,MAX),
	SUM = FIX_VALUE+(MAX-MIN+1)*MOD_VALUE.


% see traveling salespersons problem for details
%order(X,C1,C2) :- outcost(X,(C1;C2)), C1 < C2, C <= C1 : outcost(X,C), C < C2.
order(k,SYSTEM,METADATA,INFO,C1,C2) :- 
	%metadata(information;stream,METADATA,_),
	heuristic(k,tstream(k,SYSTEM,_,INFO),METADATA,(C1;C2)),
	C1 < C2, 
	C <= C1 : heuristic(k,tstream(k,SYSTEM,_,INFO),METADATA,C), C < C2.

%penalty(X,C1,C2-C1) :- order(X,C1,C2), cycle(X,Y), cost(X,Y,C2).
penalty(k,SYSTEM,METADATA,INFO,C2,C2-C1) :-
	%metadata(information;stream,METADATA,_), 
	order(k,SYSTEM,METADATA,INFO,C1,C2), 
	flagged(k,TNODE,_), tstream(k,SYSTEM,TNODE,INFO),
	heuristic(k,tstream(k,SYSTEM,TNODE,INFO),METADATA,C1).

%penalty(X,C1,C2-C1) :- order(X,C1,C2), penalty(X,C2,_).
penalty(k,SYSTEM,METADATA,INFO,C2,C2-C1) :- 
	order(k,SYSTEM,METADATA,INFO,C1,C2), 
	penalty(k,SYSTEM,METADATA,INFO,C1,_).

% TODO add configurable settings for different metadata
#minimize { D@3,SYSTEM,METADATA,INFO,C : penalty(k,SYSTEM,METADATA,INFO,C,D) }.









