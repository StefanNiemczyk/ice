%%%%--------------------------------------------------------------------------------------
%%%%
%%%% Local optimization 
%%%% 
%%%% Implements a simple local search based on penalties for not optimal streams
%%%%
%%%%--------------------------------------------------------------------------------------
#program query(k,maxHopCount,maxStepCount).

% heuristic of node output QoS
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


% see raveling salespersons problem for details
%order(X,C1,C2) :- outcost(X,(C1;C2)), C1 < C2, C <= C1 : outcost(X,C), C < C2.
order(k,SYSTEM,INFO,C1,C2) :- tstream(k,SYSTEM,_,INFO,(C1;C2)), C1 < C2, C <= C1 : tstream(k,SYSTEM,_,INFO,C), C < C2.

%penalty(X,C1,C2-C1) :- order(X,C1,C2), cycle(X,Y), cost(X,Y,C2).
penalty(k,SYSTEM,INFO,C2,C2-C1) :- order(k,SYSTEM,INFO,C1,C2), use(k,SYSTEM,NODE,ENTITY,ENTITY2,S), tstream(k,SYSTEM,tnode(k,SYSTEM,NODE,ENTITY,ENTITY2),INFO,C1).

%penalty(X,C1,C2-C1) :- order(X,C1,C2), penalty(X,C2,_).
penalty(k,SYSTEM,INFO,C2,C2-C1) :- order(k,SYSTEM,INFO,C1,C2), penalty(k,SYSTEM,INFO,C1,_).

% TODO add configurable settings for different metadata
#minimize { D@3,SYSTEM,INFO,C : penalty(k,SYSTEM,INFO,C,D) }.









