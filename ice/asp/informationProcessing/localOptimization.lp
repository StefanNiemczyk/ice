%%%%--------------------------------------------------------------------------------------
%%%%
%%%% Local optimization 
%%%% 
%%%% Implements a simple local search based on penalties for not optimal streams
%%%%
%%%%--------------------------------------------------------------------------------------
#program query(k,maxHopCount,maxStepCount).

% heuristic of node output QoS

heuristic(k,SYSTEM,NODE,0) :-
	sourceNode(SYSTEM,NODE,_).

heuristic(k,SYSTEM,NODE,0) :-
	nodeTemplate(SYSTEM,NODE,_).

heuristic(k,SYSTEM,NODE,0) :-
	iro(SYSTEM,NODE,_,_).

heuristic(k,SYSTEM,NODE,0) :-
	mapNodeTemplate(SYSTEM,NODE,_).
%*
heuristic(k,SYSTEM,NODE,MAXIMIZE-MINIMIZE) :-
	sourceNode(SYSTEM,NODE,_),
	MINIMIZE = #sum { SUM,METADATA : 
		metadata(METADATA,minimize),
		metadataNode(METADATA,SYSTEM,NODE,TYPE,FIX_VALUE,MOD_VALUE),
		nodeCount(k,SYSTEM,NODE,MIN,MAX),
		SUM = FIX_VALUE+(MAX-MIN+1)*MOD_VALUE
	},
	MAXIMIZE = #sum { SUM,METADATA : 
		metadata(METADATA,maximize), 
		metadataNode(METADATA,SYSTEM,NODE,TYPE,FIX_VALUE,MOD_VALUE),
		nodeCount(k,SYSTEM,NODE,MIN,MAX),
		SUM = FIX_VALUE+(MAX-MIN+1)*MOD_VALUE	
	}.

heuristic(k,SYSTEM,NODE,MAXIMIZE-MINIMIZE) :-
	nodeTemplate(SYSTEM,NODE,_),
	MINIMIZE = #sum { SUM,METADATA : 
		metadata(METADATA,minimize),
		metadataNode(METADATA,SYSTEM,NODE,TYPE,FIX_VALUE,MOD_VALUE),
		nodeCount(k,SYSTEM,NODE,MIN,MAX),
		SUM = FIX_VALUE+(MAX-MIN+1)*MOD_VALUE
	},
	MAXIMIZE = #sum { SUM,METADATA : 
		metadata(METADATA,maximize),
		metadataNode(METADATA,SYSTEM,NODE,TYPE,FIX_VALUE,MOD_VALUE),
		nodeCount(k,SYSTEM,NODE,MIN,MAX),
		SUM = FIX_VALUE+(MAX-MIN+1)*MOD_VALUE
	}.

heuristic(k,SYSTEM,NODE,MAXIMIZE-MINIMIZE) :-
	iro(SYSTEM,NODE,_,_),
	MINIMIZE = #sum { SUM,METADATA : 
		metadata(METADATA,minimize),
		metadataNode(METADATA,SYSTEM,NODE,TYPE,FIX_VALUE,MOD_VALUE),
		nodeCount(k,SYSTEM,NODE,MIN,MAX),
		SUM = FIX_VALUE+(MAX-MIN+1)*MOD_VALUE
	},
	MAXIMIZE = #sum { SUM,METADATA : 
		metadata(METADATA,maximize),
		metadataNode(METADATA,SYSTEM,NODE,TYPE,FIX_VALUE,MOD_VALUE),
		nodeCount(k,SYSTEM,NODE,MIN,MAX),
		SUM = FIX_VALUE+(MAX-MIN+1)*MOD_VALUE
	}.
*%

heuristic(k,tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO,C1),METADATA,1000-SUM) :-
	metadata(METADATA,minimize),
	tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO,C1),
	metadataNode(METADATA,SYSTEM2,NODE,TYPE,FIX_VALUE,MOD_VALUE),
	nodeCount(k,SYSTEM2,NODE,MIN,MAX),
	SUM = FIX_VALUE+(MAX-MIN+1)*MOD_VALUE.

heuristic(k,tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO,C1),METADATA,SUM) :-
	metadata(METADATA,maximize),
	tstream(k,SYSTEM,tnode(k,SYSTEM2,NODE,ENTITY,ENTITY2),INFO,C1),
	metadataNode(METADATA,SYSTEM2,NODE,TYPE,FIX_VALUE,MOD_VALUE),
	nodeCount(k,SYSTEM2,NODE,MIN,MAX),
	SUM = FIX_VALUE+(MAX-MIN+1)*MOD_VALUE.


% see raveling salespersons problem for details
%order(X,C1,C2) :- outcost(X,(C1;C2)), C1 < C2, C <= C1 : outcost(X,C), C < C2.
order(k,SYSTEM,METADATA,INFO,C1,C2) :- 
	metadata(METADATA,_),
	heuristic(k,tstream(k,SYSTEM,_,INFO,_),METADATA,(C1;C2)),
	C1 < C2, 
	C <= C1 : heuristic(k,tstream(k,SYSTEM,_,INFO,_),METADATA,C), C < C2.

%penalty(X,C1,C2-C1) :- order(X,C1,C2), cycle(X,Y), cost(X,Y,C2).
penalty(k,SYSTEM,METADATA,INFO,C2,C2-C1) :-
	metadata(METADATA,_), 
	order(k,SYSTEM,METADATA,INFO,C1,C2), 
	flagged(k,TNODE,_), tstream(k,SYSTEM,TNODE,INFO,_),
	heuristic(k,tstream(k,SYSTEM,TNODE,INFO,_),METADATA,C1).

%penalty(X,C1,C2-C1) :- order(X,C1,C2), penalty(X,C2,_).
penalty(k,SYSTEM,METADATA,INFO,C2,C2-C1) :- 
	order(k,SYSTEM,METADATA,INFO,C1,C2), 
	penalty(k,SYSTEM,METADATA,INFO,C1,_).

% TODO add configurable settings for different metadata
#minimize { D@3,SYSTEM,METADATA,INFO,C : penalty(k,SYSTEM,METADATA,INFO,C,D) }.









