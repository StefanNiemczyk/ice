simpleIro(NODE,SCOPE,REP1,REP2) :-
	iro(SYSTEM,NODE,any,none),
	input(SYSTEM,NODE,SCOPE,REP1,none,1,1),
	output(SYSTEM,NODE,SCOPE,REP2,none),
	INPUTS = #count{SCOPE2,REP,ENTITY2,MIN,MAX : input(SYSTEM,NODE,SCOPE2,REP,ENTITY2,MIN,MAX)},
	OUTPUTS = #count{SCOPE2,REP,ENTITY2 : output(SYSTEM,NODE,SCOPE2,REP,ENTITY2)},
	INPUTS = 1,
	OUTPUTS = 1.

hasRelatedEntity(REP) :-
	representation(REP),
	hasRelatedDimension(REP,_,_,_,_).

simpleRep(REP) :-
	representation(REP),
	not hasRelatedEntity(REP).


% determine similar representations
simRep(SCOPE,REP1,REP2) :-
	not hasRelatedEntity(REP1),
	not hasRelatedEntity(REP2), 
	hasRepresentation(SCOPE,REP1), 
	hasRepresentation(SCOPE,REP2), 
	scope(SCOPE), 
	REP1 != REP2,
	not simpleIro(_,SCOPE,REP1,REP2).

% dimension matching
match(simRep(SCOPE,REP1,REP2),DIM,DIM_REP) :-
	simRep(SCOPE,REP1,REP2),
	hasDimension(REP1,DIM,DIM_REP,MIN,MAX),
	hasDimension(REP2,DIM,DIM_REP,MIN,MAX).


% diviations

% missing dimension in target representation
dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,DIM_REP,miss) :-
	simRep(SCOPE,REP1,REP2),
	hasDimension(REP1,DIM,DIM_REP,_,_),
	not hasDimension(REP2,DIM,DIM_REP,_,_).

dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,DIM_REP,miss) :-
	simRep(SCOPE,REP1,REP2),
	hasDimension(REP1,DIM,DIM_REP,COUNT,COUNT1),
	hasDimension(REP2,DIM,DIM_REP,COUNT2,COUNT2),
	COUNT2 < COUNT1.

% missing dimension in source representation
dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,DIM_REP,empty) :-
	simRep(SCOPE,REP1,REP2),
	not hasDimension(REP1,DIM,DIM_REP,_,_),
	hasDimension(REP2,DIM,DIM_REP,_,_).

dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,DIM_REP,empty) :-
	simRep(SCOPE,REP1,REP2),
	hasDimension(REP1,DIM,DIM_REP,COUNT1,COUNT1),
	hasDimension(REP2,DIM,DIM_REP,COUNT2,COUNT2),
	COUNT1 < COUNT2.

% try to fix if at least 50% dimensions are matching
tryFix(simRep(SCOPE,REP1,REP2),MATCHING) :-
	simRep(SCOPE,REP1,REP2),
	MATCHING = #count{DIM,DIM_REP : match(simRep(SCOPE,REP1,REP2),DIM,DIM_REP)},
	MATCHING > 0.


% fixes
% using exisitng iro to transform dimensions
{ useIro(simRep(SCOPE,REP1,REP2),SCOPE,REP_SOURCE,REP_TARGET,NODE) } :-
	tryFix(simRep(SCOPE,REP1,REP2),_),
	dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,REP_SOURCE,miss),
	dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,REP_TARGET,empty),
%	hasRepresentation(SCOPE,REP_SOURCE),
%	hasRepresentation(SCOPE,REP_TARGET),
	simpleIro(NODE,DIM,REP_SOURCE,REP_TARGET).

fix(SIM_REP,DIM,REP_SOURCE,iro) :-
	useIro(SIM_REP,DIM,REP_SOURCE,REP_TARGET,NODE).

fix(SIM_REP,DIM,REP_TARGET,iro) :-
	useIro(SIM_REP,DIM,REP_SOURCE,REP_TARGET,NODE).

fixCost(SIM_REP,useIro(SIM_REP,DIM,REP_SOURCE,REP_TARGET,NODE),10) :-
	useIro(SIM_REP,DIM,REP_SOURCE,REP_TARGET,NODE).


% using already computed transformations to transform dimensions
useAutoIRO(SIM_REP,DIM,REP_SOURCE,REP_TARGET)  :-
	tryFix(simRep(SCOPE,REP1,REP2),_),
	dimensionDeviation(SIM_REP,DIM,REP_SOURCE,miss),
	dimensionDeviation(SIM_REP,DIM,REP_TARGET,empty),
	autoIRO(DIM,REP_SOURCE,REP_TARGET).

fix(SIM_REP,DIM,REP_SOURCE,autoIro) :-
	useAutoIRO(SIM_REP,DIM,REP_SOURCE,REP_TARGET).
fix(SIM_REP,DIM,REP_TARGET,autoIro) :-
	useAutoIRO(SIM_REP,DIM,REP_SOURCE,REP_TARGET).

fixCost(SIM_REP,useAutoIRO(SIM_REP,DIM,REP_SOURCE,REP_TARGET),1) :-
	useAutoIRO(SIM_REP,DIM,REP_SOURCE,REP_TARGET).


% remove missing element in target represenation
{ fix(simRep(SCOPE,REP1,REP2),DIM,DIM_REP,remove) } :-
	tryFix(simRep(SCOPE,REP1,REP2),_),
	dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,DIM_REP,miss).

fixCost(SIM_REP,fix(SIM_REP,DIM,DIM_REP,remove),10) :-
	fix(SIM_REP,DIM,DIM_REP,remove).


% use default value for empty source representation 
{ fix(SIM_REP,DIM,DIM_REP,default) } :-
	tryFix(SIM_REP,_),
	dimensionDeviation(SIM_REP,DIM,DIM_REP,empty),
	valueScopeDefault(DIM,_).

fixCost(SIM_REP,fix(SIM_REP,DIM,DIM_REP,default),10) :-
	fix(SIM_REP,DIM,DIM_REP,default).


% resulting auto iros
autoIRO(SCOPE,REP1,REP2) :-
	tryFix(simRep(SCOPE,REP1,REP2),_),
	not dimensionDeviation(simRep(SCOPE,REP1,REP2),_,_,_).

fixed(simRep(SCOPE,REP1,REP2)) :-
	tryFix(simRep(SCOPE,REP1,REP2),_),
	fix(simRep(SCOPE,REP1,REP2),DIM,DIM_REP,_) : dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,DIM_REP,_).

autoIRO(SCOPE,REP1,REP2) :- 
	tryFix(simRep(SCOPE,REP1,REP2),_),
	fixed(simRep(SCOPE,REP1,REP2)).


% prevent constraints 
sumFixCost(SIM_REP,SUM) :-
	tryFix(SIM_REP,_),
	SUM = #sum{COST,FIX : fixCost(SIM_REP,FIX,COST)},
	SUM > 0.
:- tryFix(SIM_REP,MATCHING), sumFixCost(SIM_REP,SUM), SUM >= 10 * MATCHING.


% maximize the number of created auto iros
#maximize{ 1@2,SCOPE,REP1,REP2 : autoIRO(SCOPE,REP1,REP2) }.

% minimize the number of fixing steps
#minimize{ COST@1,SIM_REP : sumFixCost(SIM_REP,COST) }.

















