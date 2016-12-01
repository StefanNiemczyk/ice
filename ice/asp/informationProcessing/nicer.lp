%%%%--------------------------------------------------------------------------------------
%%%%
%%%% NICER Example
%%%%
%%%% Implements the communication infrastructure of NICER
%%%%
%%%%--------------------------------------------------------------------------------------
#program query(k,maxHopCount,maxStepCount).

% compute transfer term based on islands and bridges
transfer(SYSTEM_SOURCE,SYSTEM_TARGET) :- 
	bridge(ISLAND1,ISLAND2),
	connectToBridge(SYSTEM_SOURCE,ISLAND2),
	connectToBridge(SYSTEM_TARGET,ISLAND1),
	system(SYSTEM_SOURCE,ISLAND1), 
	system(SYSTEM_TARGET,ISLAND2).

metadataProcessing(METADATA,SYSTEM_SOURCE,SYSTEM_TARGET,VALUE1+VALUE2+VALUE3) :- 
	bridge(ISLAND1,ISLAND2), 
	system(SYSTEM_SOURCE,ISLAND1), 
	system(SYSTEM_TARGET,ISLAND2),
	connectToBridge(SYSTEM_SOURCE,ISLAND2),
	connectToBridge(SYSTEM_TARGET,ISLAND1),
	metadataProcessing(METADATA,SYSTEM_SOURCE,ISLAND2,VALUE1),
	metadataProcessing(METADATA,SYSTEM_TARGET,ISLAND1,VALUE2),
	metadataProcessing(METADATA,ISLAND1,ISLAND2,VALUE3).

metadataOutput(METADATA,SYSTEM_SOURCE,SYSTEM_TARGET,VALUE1+VALUE2+VALUE3) :- 
	bridge(ISLAND1,ISLAND2), 
	system(SYSTEM_SOURCE,ISLAND1), 
	system(SYSTEM_TARGET,ISLAND2),
	connectToBridge(SYSTEM_SOURCE,ISLAND2),
	connectToBridge(SYSTEM_TARGET,ISLAND1),
	metadataOutput(METADATA,SYSTEM_SOURCE,ISLAND2,VALUE1),
	metadataOutput(METADATA,SYSTEM_TARGET,ISLAND1,VALUE2),
	metadataOutput(METADATA,ISLAND1,ISLAND2,VALUE3).


% metadata computation: islandCoverage
% definition
metadata(stream,islandCoverage,maximize).
metadata(set,islandCoverage,maximize).

% Propagation rule
metadataSet(k,islandCoverage,set(k,SYSTEM,setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP),VALUE+VALUE_SUM) :- 
	set(k,SYSTEM,setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),INFO_TYPE,STEP), 
	metadataOutput(islandCoverage,SYSTEM,NODE,sum,_,_),
	VALUE = #count{ ISLAND : 
		connectToSet(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),stream(k,_,node(_,SYSTEM2,_,_,_),_,_)), 
		system(SYSTEM2,ISLAND)}, 
	VALUE_SUM = #sum{D,SET_INPUT : 
		connectToSet(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),SET_INPUT),
		metadataSet(k,islandCoverage,SET_INPUT,D)}, 
	query(k).

% optimization
#maximize { VALUE@4,SELECTED : metadataSelected(islandCoverage,SELECTED,VALUE) }.

%--------------------------------------------------------------------------------------
#program island(name).

#external island(name).


%--------------------------------------------------------------------------------------
#program system(name,points,island).

#external system(name,island).
systemPoints(name,points) :- system(name,island).


%--------------------------------------------------------------------------------------
#program connectToBridge(system,island,delayValue,costValue).

connectToBridge(system,island).
metadataProcessing(cost,system,island,costValue).
metadataOutput(delay,system,island,delayValue).


%--------------------------------------------------------------------------------------
#program bridge(island1,island2,delayValue,costValue).

#external bridge(island1,island2).
bridge(island2,island1) :- bridge(island1,island2).

metadataProcessing(cost,island1,island2,costValue).
metadataOutput(delay,island1,island2,delayValue).

metadataProcessing(cost,island2,island1,costValue).
metadataOutput(delay,island2,island1,delayValue).
