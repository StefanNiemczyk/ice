
% Identify usable transform operations
simpleTrans(NODE,SCOPE,REP1,REP2) :-
	transformation(SYSTEM,NODE,any,none),
	input(SYSTEM,NODE,SCOPE,REP1,none,1,1),
	output(SYSTEM,NODE,SCOPE,REP2,none).
	%INPUTS = #count{SCOPE2,REP,ENTITY2,MIN,MAX : input(SYSTEM,NODE,SCOPE2,REP,ENTITY2,MIN,MAX)},
	%OUTPUTS = #count{SCOPE2,REP,ENTITY2 : output(SYSTEM,NODE,SCOPE2,REP,ENTITY2)},
	%INPUTS = 1,
	%OUTPUTS = 1.

-simpleRep(REP) :-
	representation(REP),
	hasRelatedDimension(REP,_,_).

simpleRep(REP) :-
	representation(REP),
	not -simpleRep(REP).

% determine similar representations
%simRep(SCOPE,REP1,REP2) :-
%	simpleRep(REP1),
%	simpleRep(REP2), 
%	hasRepresentation(SCOPE,REP1), 
%	hasRepresentation(SCOPE,REP2), 
%	scope(SCOPE), 
%	REP1 != REP2.

% dimension matching
%match(simRep(SCOPE,REP1,REP2),DIM,DIM_REP) :-
%	autoTrans(SCOPE,REP1,REP2),
%	hasDimension(REP1,DIM,DIM_REP),
%	hasDimension(REP2,DIM,DIM_REP).


%% try to fix if at least one dimension is matching
tryFix(simRep(SCOPE,REP1,REP2)) :-
	simpleRep(REP1),
	simpleRep(REP2), 
	hasRepresentation(SCOPE,REP1), 
	hasRepresentation(SCOPE,REP2), 
	scope(SCOPE), 
	REP1 != REP2,
	hasDimension(REP1,DIM,_),
	hasDimension(REP2,DIM,_),
	not simpleTrans(_,SCOPE,REP1,REP2).


%% diviations

% missing dimension in target representation
dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,DIM_REP,missing) :-
	tryFix(simRep(SCOPE,REP1,REP2)),
	hasDimension(REP1,DIM,DIM_REP),
	not hasDimension(REP2,DIM,DIM_REP).

% missing dimension in source representation
dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,DIM_REP,empty) :-
	tryFix(simRep(SCOPE,REP1,REP2)),
	not hasDimension(REP1,DIM,DIM_REP),
	hasDimension(REP2,DIM,DIM_REP).


%% fixes

% using exisitng transformation to transform dimensions
{ useTrans(simRep(SCOPE,REP1,REP2),SCOPE,REP_SOURCE,REP_TARGET,NODE) } :-
	tryFix(simRep(SCOPE,REP1,REP2)),
	dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,REP_SOURCE,missing),
	dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,REP_TARGET,empty),
	simpleTrans(NODE,DIM,REP_SOURCE,REP_TARGET).

fixed(SIM_REP,DIM,REP_SOURCE,transformation) :-
	useTrans(SIM_REP,DIM,REP_SOURCE,REP_TARGET,NODE).

fixed(SIM_REP,DIM,REP_TARGET,transformation) :-
	useTrans(SIM_REP,DIM,REP_SOURCE,REP_TARGET,NODE).

fixCost(SIM_REP,useTrans(SIM_REP,DIM,REP_SOURCE,REP_TARGET,NODE),10) :-
	useTrans(SIM_REP,DIM,REP_SOURCE,REP_TARGET,NODE).


% using already computed transformations to transform dimensions
{ useAutoTrans(SIM_REP,DIM,REP_SOURCE,REP_TARGET) } :-
	tryFix(SIM_REP),
	dimensionDeviation(SIM_REP,DIM,REP_SOURCE,missing),
	dimensionDeviation(SIM_REP,DIM,REP_TARGET,empty),
	autoTrans(DIM,REP_SOURCE,REP_TARGET).

fixed(SIM_REP,DIM,REP_SOURCE,autoTrans) :-
	useAutoTrans(SIM_REP,DIM,REP_SOURCE,REP_TARGET).
fixed(SIM_REP,DIM,REP_TARGET,autoTrans) :-
	useAutoTrans(SIM_REP,DIM,REP_SOURCE,REP_TARGET).

fixCost(SIM_REP,useAutoTrans(SIM_REP,DIM,REP_SOURCE,REP_TARGET),1) :-
	useAutoTrans(SIM_REP,DIM,REP_SOURCE,REP_TARGET).


% remove missing element in target represenation
{ fixed(simRep(SCOPE,REP1,REP2),DIM,DIM_REP,remove) } :-
	tryFix(simRep(SCOPE,REP1,REP2)),
	dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,DIM_REP,missing).

fixCost(SIM_REP,fixed(SIM_REP,DIM,DIM_REP,remove),10) :-
	fixed(SIM_REP,DIM,DIM_REP,remove).


% use default value for empty source representation 
{ fixed(SIM_REP,DIM,DIM_REP,default) } :-
	tryFix(SIM_REP),
	dimensionDeviation(SIM_REP,DIM,DIM_REP,empty),
	valueScopeDefault(DIM,_).

fixCost(SIM_REP,fixed(SIM_REP,DIM,DIM_REP,default),5) :-
	fixed(SIM_REP,DIM,DIM_REP,default).


% identify generated transformations
-fixed(SIM_REP) :-
	not fixed(SIM_REP,DIM,DIM_REP,_), 
	dimensionDeviation(SIM_REP,DIM,DIM_REP,_).

autoTrans(SCOPE,REP1,REP2) :- 
	tryFix(simRep(SCOPE,REP1,REP2)),
	not -fixed(simRep(SCOPE,REP1,REP2)).


% maximize the number of created auto transformations
#maximize{ 1@2,SCOPE,REP1,REP2 : autoTrans(SCOPE,REP1,REP2) }.

% minimize the number of fixing steps
#minimize{ COST@1,SCOPE,REP1,REP2,FIX : fixCost(simRep(SCOPE,REP1,REP2),FIX,COST), autoTrans(SCOPE,REP1,REP2) }.


















