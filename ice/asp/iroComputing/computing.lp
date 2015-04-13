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

hasRelatedEntity(REP) :-
	representation(REP),
	hasRelatedDimension(REP,_,_,_).

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
%simple
match(simRep(SCOPE,REP1,REP2),DIM) :-
	simRep(SCOPE,REP1,REP2),
	hasDimension(REP1,DIM,MIN,MAX),
	hasDimension(REP2,DIM,MIN,MAX).

%complex
match(simRep(SCOPE,REP1,REP2),SCOPE,REP) :-
	simRep(SCOPE,REP1,REP2),
	hasDimension(REP1,SCOPE,REP,MIN,MAX),
	hasDimension(REP2,SCOPE,REP,MIN,MAX).


% diviations

% missing dimension in target representation
%simple
dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,miss) :-
	simRep(SCOPE,REP1,REP2),
	hasDimension(REP1,DIM,_,_),
	not hasDimension(REP2,DIM,_,_).

dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,miss) :-
	simRep(SCOPE,REP1,REP2),
	hasDimension(REP1,DIM,COUNT,COUNT1),
	hasDimension(REP2,DIM,COUNT2,COUNT2),
	COUNT2 < COUNT1.

%complex
dimensionDeviation(simRep(SCOPE,REP1,REP2),SCOPE,REP,miss) :-
	simRep(SCOPE,REP1,REP2),
	hasDimension(REP1,SCOPE,REP,_,_),
	not hasDimension(REP2,SCOPE,REP,_,_).

dimensionDeviation(simRep(SCOPE,REP1,REP2),SCOPE,REP,miss) :-
	simRep(SCOPE,REP1,REP2),
	hasDimension(REP1,SCOPE,REP,COUNT,COUNT1),
	hasDimension(REP2,SCOPE,REP,COUNT2,COUNT2),
	COUNT2 < COUNT1.


% missing dimension in source representation
%simple
dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,empty) :-
	simRep(SCOPE,REP1,REP2),
	not hasDimension(REP1,DIM,_,_),
	hasDimension(REP2,DIM,_,_).

dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,empty) :-
	simRep(SCOPE,REP1,REP2),
	hasDimension(REP1,DIM,COUNT,COUNT1),
	hasDimension(REP2,DIM,COUNT2,COUNT2),
	COUNT1 < COUNT2.

%complex
dimensionDeviation(simRep(SCOPE,REP1,REP2),SCOPE,REP,empty) :-
	simRep(SCOPE,REP1,REP2),
	not hasDimension(REP1,SCOPE,REP,_,_),
	hasDimension(REP2,SCOPE,REP,_,_).

dimensionDeviation(simRep(SCOPE,REP1,REP2),SCOPE,REP,empty) :-
	simRep(SCOPE,REP1,REP2),
	hasDimension(REP1,SCOPE,REP,COUNT1,COUNT1),
	hasDimension(REP2,SCOPE,REP,COUNT2,COUNT2),
	COUNT1 < COUNT2.


% try to fix if at least 50% dimensions are matching
tryFix(simRep(SCOPE,REP1,REP2),MATCHING) :-
	simRep(SCOPE,REP1,REP2),
	MATCHING = #count{DIM : match(simRep(SCOPE,REP1,REP2),DIM)},
	%COUNT1 = #count{REP,COUNT,COUNT1 : hasDimension(REP1,SCOPE,REP,COUNT,COUNT1)},
	%COUNT2 = #count{REP,COUNT,COUNT1 : hasDimension(REP2,SCOPE,REP,COUNT,COUNT1)},
	%COUNT1 > COUNT2,
	%MATCHING >= COUNT1,
	MATCHING > 0.

tryFix(simRep(SCOPE,REP1,REP2),MATCHING) :-
	simRep(SCOPE,REP1,REP2),
	MATCHING = #count{DIM : match(simRep(SCOPE,REP1,REP2),DIM)},
	%COUNT1 = #count{REP,COUNT,COUNT1 : hasDimension(REP1,SCOPE,REP,COUNT,COUNT1)},
	%COUNT2 = #count{REP,COUNT,COUNT1 : hasDimension(REP2,SCOPE,REP,COUNT,COUNT1)},
	%COUNT1 <= COUNT2,
	%MATCHING >= COUNT2,
	MATCHING > 0.


% fixes

% using exisitng iro to transform complex dimensions
{ useIro(simRep(SCOPE,REP1,REP2),SCOPE,REP_SOURCE,REP_TARGET,NODE) } :-
	tryFix(simRep(SCOPE,REP1,REP2),_),
	dimensionDeviation(simRep(SCOPE,REP1,REP2),SCOPE,REP_SOURCE,miss),
	dimensionDeviation(simRep(SCOPE,REP1,REP2),SCOPE,REP_TARGET,empty),
	simpleIro(NODE,SCOPE,REP_SOURCE,REP_TARGET).

fix(simRep(SCOPE,REP1,REP2),SCOPE,REP_SOURCE,iro), fix(simRep(SCOPE,REP1,REP2),SCOPE,REP_TARGET,iro) :-
	useIro(simRep(SCOPE,REP1,REP2),SCOPE,REP_SOURCE,REP_TARGET,NODE).

fixCost(SIM_REP,useIro(SIM_REP,SCOPE,REP_SOURCE,REP_TARGET,NODE),10) :-
	useIro(SIM_REP,SCOPE,REP_SOURCE,REP_TARGET,NODE).


% remove missing element in target represenation
%simple
{ fix(simRep(SCOPE,REP1,REP2),DIM,remove) } :-
	tryFix(simRep(SCOPE,REP1,REP2),_),
	dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,miss).

fixCost(SIM_REP,fix(SIM_REP,DIM,remove),10) :-
	fix(SIM_REP,DIM,remove).

%complex
{ fix(SIM_REP,SCOPE,REP,remove) } :-
	tryFix(SIM_REP,_),
	dimensionDeviation(SIM_REP,SCOPE,REP,miss).

fixCost(SIM_REP,fix(SIM_REP,SCOPE,REP,remove),10) :-
	fix(SIM_REP,SCOPE,REP,remove).


% use default value for empty source representation 
%simple
{ fix(SIM_REP,DIM,default) } :-
	tryFix(SIM_REP,_),
	dimensionDeviation(SIM_REP,DIM,empty),
	valueScopeDefault(DIM,_).

fixCost(SIM_REP,fix(SIM_REP,DIM,default),10) :-
	fix(SIM_REP,DIM,default).

%complex
{ fix(SIM_REP,SCOPE,REP,default) } :-
	tryFix(SIM_REP,_),
	dimensionDeviation(SIM_REP,SCOPE,REP,empty),
	valueScopeDefault(SCOPE,REP,_).

fixCost(SIM_REP,fix(SIM_REP,SCOPE,REP,default),10) :-
	fix(SIM_REP,SCOPE,REP,default).

% resulting auto iros
autoIRO(SCOPE,REP1,REP2) :-
	tryFix(simRep(SCOPE,REP1,REP2),_),
	not dimensionDeviation(simRep(SCOPE,REP1,REP2),_,_),
	not dimensionDeviation(simRep(SCOPE,REP1,REP2),_,_,_).

fixedSimple(simRep(SCOPE,REP1,REP2)) :-
	tryFix(simRep(SCOPE,REP1,REP2),_),
	fix(simRep(SCOPE,REP1,REP2),DIM,_) : dimensionDeviation(simRep(SCOPE,REP1,REP2),DIM,_).

fixedComplex(simRep(SCOPE,REP1,REP2)) :-
	tryFix(simRep(SCOPE,REP1,REP2),_),
	fix(simRep(SCOPE,REP1,REP2),SCOPE,REP,_) : dimensionDeviation(simRep(SCOPE,REP1,REP2),SCOPE,REP,_).

autoIRO(SCOPE,REP1,REP2) :- 
	tryFix(simRep(SCOPE,REP1,REP2),_),
	fixedSimple(simRep(SCOPE,REP1,REP2)),
	fixedComplex(simRep(SCOPE,REP1,REP2)).


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

















