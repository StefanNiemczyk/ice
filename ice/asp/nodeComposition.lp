% information/3		:		(entity,scope,representation)
% stream/3		: 		(system,source,information)
% node/3		:		(system,nodeName,entity)
% output/6 		:		(system,node,scope,rep,acc_mode,acc_modifier)
%--------------------------------------------------------------------------------------
% paramters
 #const priorityDelay=3.
 #const priorityAccuracy=3.
 #const priorityTransferCost=2.
 #const priorityCost=1.
 #const maxTransferDelayValue=100.
 #const maxTranslateDelayValue=100.
 #const minAccuracyValue=0.
 #const maxAccuracyValue=100000.
 

%--------------------------------------------------------------------------------------
#program base.
%#show streamDelay/5.
%#show nodeTemplate/2.
%#show transfer/4.
%#show requiredStream/4.
%#show system/2.

%#show node/4.
%#show stream/5.
%#show connectToNode/6.
%#show streamTransfer/7.
%--------------------------------------------------------------------------------------
#program query(k).

#external query(k).

% preprocessing
%{ stream(k,SYSTEM,SOURCE,INFO) } :- sourceNode(SYSTEM,SOURCE,INFO,_,_,_), query(k).
%streamDelay(k,SYSTEM,SOURCE,INFO,DELAY) :- sourceNode(SYSTEM,SOURCE,INFO,DELAY,_,_), stream(k,SYSTEM,SOURCE,INFO).
%streamAccuracy(k,SYSTEM,SOURCE,INFO,ACCURACY) :- sourceNode(SYSTEM,SOURCE,INFO,_,ACCURACY,_), stream(k,SYSTEM,SOURCE,INFO).
%streamCost(k,SYSTEM,SOURCE,INFO,COST) :- sourceNode(SYSTEM,SOURCE,INFO,_,_,COST), stream(k,SYSTEM,SOURCE,INFO).


%%% guessing

%% creating a node
0 { node(k,SYSTEM,NODE,ENTITY) } 1 :- sourceNode(SYSTEM,NODE,ENTITY), entity(ENTITY,_), query(k).
0 { node(k,SYSTEM,NODE,ENTITY) } 1 :- nodeTemplate(SYSTEM,NODE,ENTITY_TYPE), entity(ENTITY,ENTITY_TYPE), query(k).
%0 { node(k,SYSTEM,NODE,ENTITY) } 1 :- nodeTemplate(SYSTEM,NODE,any), entity(ENTITY,_), query(k).
0 { node(k,SYSTEM,NODE,ENTITY) } 1 :- nodeTemplate(SYSTEM,NODE,any), entity(ENTITY,ENTITY_TYPE), hasScope(ENTITY_TYPE,SCOPE), input(SYSTEM,NODE,SCOPE,_,_,_), query(k).
MIN { connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none)) : stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none)), PROVIDER != NODE } MAX :- 
		input(SYSTEM,NODE,SCOPE,REP,MIN,MAX), node(k,SYSTEM,NODE,ENTITY).

stream(k,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none)) :- node(k,SYSTEM,NODE,ENTITY), output(SYSTEM,NODE,SCOPE,REP).

% computing QoS paramters
streamDelay(k,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none),ND) :- node(k,SYSTEM,NODE,ENTITY), output(SYSTEM,NODE,SCOPE,REP), nodeDelayFix(SYSTEM,NODE,ND).
streamDelay(k,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none),ND) :- node(k,SYSTEM,NODE,ENTITY), output(SYSTEM,NODE,SCOPE,REP), nodeDelay(SYSTEM,NODE,ND),
		SD = #max{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), streamDelay(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, SD==#infimum.
streamDelay(k,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none),ND+SD) :- node(k,SYSTEM,NODE,ENTITY), output(SYSTEM,NODE,SCOPE,REP), nodeDelay(SYSTEM,NODE,ND),
		SD = #max{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), streamDelay(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, SD!=#infimum.

streamAccuracy(k,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none),ND) :- node(k,SYSTEM,NODE,ENTITY), output(SYSTEM,NODE,SCOPE,REP), nodeAccuracyFix(SYSTEM,NODE,ND).
streamAccuracy(k,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none),ACC_COUNT*ACC_MODIFIER+ACC) :- node(k,SYSTEM,NODE,ENTITY), output(SYSTEM,NODE,SCOPE,REP), 
		nodeAccuracyMin(SYSTEM,NODE,ACC_MODIFIER), ACC = #min{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
		ACC_COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)},
		ACC_MODIFIER+ACC < maxAccuracyValue, ACC_MODIFIER+ACC > minAccuracyValue.
streamAccuracy(k,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none),ACC_COUNT*ACC_MODIFIER+ACC) :- node(k,SYSTEM,NODE,ENTITY), output(SYSTEM,NODE,SCOPE,REP), 
		nodeAccuracyAvg(SYSTEM,NODE,ACC_MODIFIER), ACC_SUM = #sum{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
		ACC_COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, ACC_COUNT != 0,
		ACC = ACC_SUM / ACC_COUNT, ACC_MODIFIER+ACC < maxAccuracyValue, ACC_MODIFIER+ACC > minAccuracyValue.
streamAccuracy(k,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none),0) :- node(k,SYSTEM,NODE,ENTITY), output(SYSTEM,NODE,SCOPE,REP), nodeAccuracyAvg(SYSTEM,NODE,ACC_MODIFIER), 
		ACC_COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, ACC_COUNT == 0.
streamAccuracy(k,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none),ACC_COUNT*ACC_MODIFIER+ACC) :- node(k,SYSTEM,NODE,ENTITY), output(SYSTEM,NODE,SCOPE,REP), 
		nodeAccuracyMax(SYSTEM,NODE,ACC_MODIFIER), 
		ACC = #max{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, ACC!=#infimum, 
		ACC_COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, ACC_COUNT != 0,
		ACC_COUNT*ACC_MODIFIER+ACC < maxAccuracyValue, ACC_COUNT*ACC_MODIFIER+ACC > minAccuracyValue.
streamAccuracy(k,SYSTEM,NODE,SYSTEM,information(ENTITY,SCOPE,REP,none),0) :- node(k,SYSTEM,NODE,ENTITY), output(SYSTEM,NODE,SCOPE,REP), nodeAccuracyMax(SYSTEM,NODE,ACC_MODIFIER), 
		ACC = #max{D,PROVIDER,SOURCE,INFO : connectToNode(k,SYSTEM,NODE,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, ACC==#infimum.


%% transfer a stream between systems
0 { streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,TRANSFER_DELAY,TRANSFER_COST) } 1
		:- transfer(SYSTEM_SOURCE,SYSTEM_TARGET,TRANSFER_DELAY,TRANSFER_COST), stream(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO), SYSTEM_TARGET != SOURCE.

% computing QoS paramters		
stream(k,SYSTEM_TARGET,PROVIDER,SOURCE,INFO) :- streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,TRANSFER_DELAY,TRANSFER_COST).

streamDelay(k,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,TRANSFER_DELAY+ DELAY) :- streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,TRANSFER_DELAY,TRANSFER_COST), 
		streamDelay(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,DELAY), TRANSFER_DELAY+DELAY < maxTransferDelayValue.

streamAccuracy(k,SYSTEM_TARGET,PROVIDER,SOURCE,INFO,ACCURACY) :- streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,TRANSFER_DELAY,TRANSFER_COST), 
		streamAccuracy(k,SYSTEM_SOURCE,PROVIDER,SOURCE,INFO,ACCURACY).


%% translate representation
0 { informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,none,none) } 1 :- iro(SYSTEM,IRO,any,SCOPE), entity(ENTITY,ENTITY_TYPE), hasScope(ENTITY_TYPE,SCOPE), query(k).
0 { informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,none,none) } 1 :- iro(SYSTEM,IRO,ENTITY_TYPE,SCOPE), entity(ENTITY,ENTITY_TYPE), hasScope(ENTITY_TYPE,SCOPE), query(k).
0 { informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,ENTITY2,SCOPE2) } 1 :- iro(SYSTEM,IRO,any,SCOPE,any,SCOPE2), entity(ENTITY,ENTITY_TYPE), hasScope(ENTITY_TYPE,SCOPE), 
		entity(ENTITY2,ENTITY_TYPE2), hasScope(ENTITY_TYPE2,SCOPE2), query(k).
0 { informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,ENTITY2,SCOPE2) } 1 :- iro(SYSTEM,IRO,ENTITY_TYPE,SCOPE,ENTITY_TYPE2,SCOPE2), entity(ENTITY,ENTITY_TYPE), hasScope(ENTITY_TYPE,SCOPE), 
		entity(ENTITY2,ENTITY_TYPE2), hasScope(ENTITY_TYPE2,SCOPE2), query(k).

MIN { connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none)) : stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none)), PROVIDER != IRO } MAX :- 
		inputIro(SYSTEM,IRO,SCOPE,REP,MIN,MAX), informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,_,_).
MIN { connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none)) : stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,none)), PROVIDER != IRO } MAX :- 
		inputIro2(SYSTEM,IRO,SCOPE,REP,MIN,MAX), informationTranslation(k,SYSTEM,IRO,_,_,ENTITY,SCOPE).

outputIro(SYSTEM,IRO,SCOPE,REP) :- informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,_,_), outputIro(SYSTEM,IRO,REP).

stream(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE2,REP,ENTITY2)) :- informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,ENTITY2,_), outputIro(SYSTEM,IRO,SCOPE2,REP).

% computing QoS paramters
streamDelay(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE2,REP,ENTITY2),ND) :- informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,ENTITY2,_), outputIro(SYSTEM,IRO,SCOPE2,REP), 
		iroDelay(SYSTEM,IRO,ND), SD = #max{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), streamDelay(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, SD==#infimum.
streamDelay(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE2,REP,ENTITY2),ND+SD) :- informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,ENTITY2,_), outputIro(SYSTEM,IRO,SCOPE2,REP), 
		iroDelay(SYSTEM,IRO,ND), SD = #max{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), streamDelay(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, SD!=#infimum.

streamAccuracy(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE2,REP,ENTITY2),ACC_COUNT*ACC_MODIFIER+ACC) :- informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,ENTITY2,_), outputIro(SYSTEM,IRO,SCOPE2,REP), 
		iroAccuracyMin(SYSTEM,IRO,ACC_MODIFIER), ACC = #min{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
		ACC_COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)},
		ACC_MODIFIER+ACC < maxAccuracyValue, ACC_MODIFIER+ACC > minAccuracyValue.
streamAccuracy(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE2,REP,ENTITY2),ACC_COUNT*ACC_MODIFIER+ACC) :- informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,ENTITY2,_), outputIro(SYSTEM,IRO,SCOPE2,REP), 
		iroAccuracyAvg(SYSTEM,IRO,ACC_MODIFIER), ACC_SUM = #sum{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
		ACC_COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, ACC_COUNT != 0,
		ACC = ACC_SUM / ACC_COUNT, ACC_MODIFIER+ACC < maxAccuracyValue, ACC_MODIFIER+ACC > minAccuracyValue.
streamAccuracy(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE2,REP,ENTITY2),0) :- informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,ENTITY2,_), outputIro(SYSTEM,IRO,SCOPE2,REP), 
		iroAccuracyAvg(SYSTEM,IRO,ACC_MODIFIER), ACC_COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,SOURCE,PROVIDER,INFO,D)}, 
		ACC_COUNT == 0.
streamAccuracy(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE2,REP,ENTITY2),ACC_COUNT*ACC_MODIFIER+ACC) :- informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,ENTITY2,_), 
		outputIro(SYSTEM,IRO,SCOPE2,REP), iroAccuracyMax(SYSTEM,IRO,ACC_MODIFIER), 
		ACC = #max{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, ACC!=#infimum, 
		ACC_COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, ACC_COUNT != 0,
		ACC_MODIFIER+ACC < maxAccuracyValue, ACC_MODIFIER+ACC > minAccuracyValue.
streamAccuracy(k,SYSTEM,IRO,SYSTEM,information(ENTITY,SCOPE2,REP,ENTITY2),0) :- informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE,ENTITY2,_), outputIro(SYSTEM,IRO,SCOPE2,REP), 
		iroAccuracyMax(SYSTEM,IRO,ACC_MODIFIER), ACC = #max{D,PROVIDER,SOURCE,INFO : connectToIro(k,SYSTEM,IRO,PROVIDER,SOURCE,INFO), 
		streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, ACC==#infimum.


%% extract from representation
0 { informationExtrection(k,SYSTEM,PROVIDER,SOURCE,SCOPE,REP1,information(ENTITY,SUB_SCOPE,REP2,none)) } 1 :- 
		stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP1,none)), hasDimension(REP1,SUB_SCOPE,_,_), hasRepresentation(SUB_SCOPE,REP2), query(k).
stream(k,SYSTEM,PROVIDER,SOURCE,INFO) :- informationExtrection(k,SYSTEM,PROVIDER,SOURCE,SCOPE,REP1,INFO).

% computing QoS paramters	
streamDelay(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SUB_SCOPE,REP2,ENTITY2),DELAY) :- informationExtrection(k,SYSTEM,PROVIDER,SOURCE,SCOPE,REP1,information(ENTITY,SUB_SCOPE,REP2,ENTITY2)),
		streamDelay(k,SYSTEM_SOURCE,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP1,ENTITY2),DELAY).
streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SUB_SCOPE,REP2,ENTITY2),ACCURACY) :- informationExtrection(k,SYSTEM,PROVIDER,SOURCE,SCOPE,REP1,information(ENTITY,SUB_SCOPE,REP2,ENTITY2)),
		streamAccuracy(k,SYSTEM_SOURCE,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP1,ENTITY2),ACCURACY).


%% creating a map
0 { map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2) } 1 :- mapTemplate(SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2), query(k).

MIN { connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2)) : stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2)) } MAX :- 
		map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2), inputMap(SYSTEM,MAP,MIN,MAX).

%mapDelay(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,MD) :- map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2), mapDelay(SYSTEM,MAP,MD),
%		SD = #max{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), streamDelay(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, SD==#infimum.
%mapDelay(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,MD+SD) :- map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2), mapDelay(SYSTEM,MAP,MD),
%		SD = #max{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), streamDelay(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, SD!=#infimum.

%mapAccuracy(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,ACC_COUNT*ACC_MODIFIER+ACC) :- map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2), mapAccuracy(SYSTEM,MAP,ACC_MODIFIER), 
%		ACC_SUM = #sum{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
%		ACC_COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, ACC_COUNT != 0,
%		ACC = ACC_SUM / ACC_COUNT, ACC_MODIFIER+ACC < maxAccuracyValue, ACC_MODIFIER+ACC > minAccuracyValue.
%mapAccuracy(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,0) :- map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2), mapAccuracy(SYSTEM,MAP,ACC_MODIFIER),  
%		ACC_COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
%		ACC_COUNT == 0.

metadataStream(k,delay,SYSTEM,PROVIDER,SOURCE,INFO,D) :- streamDelay(k,SYSTEM,PROVIDER,SOURCE,INFO,D).
metadataStream(k,accuracy,SYSTEM,PROVIDER,SOURCE,INFO,D) :- streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,D).

% computing QoS paramters
metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,FIX_VALUE+VALUE) :- map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2), metadataMap(METADATA,SYSTEM,MAP,min,FIX_VALUE),
		VALUE = #min{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, SD==#infimum.

metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,RESULT) :- map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2), metadataMap(METADATA,SYSTEM,MAP,avg,FIX_VALUE),
		VALUE = #sum{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, 
		COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)},
		RESULT = VALUE/COUNT+FIX_VALUE, COUNT!=0. % TODO
metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,FIX_VALUE) :- map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2), metadataMap(METADATA,SYSTEM,MAP,avg,FIX_VALUE), 
		COUNT = #count{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, COUNT==0.

metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,FIX_VALUE) :- map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2), metadataMap(METADATA,SYSTEM,MAP,max,FIX_VALUE),
		VALUE = #max{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, VALUE==#infimum.
metadataMap(k,METADATA,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,FIX_VALUE+VALUE) :- map(k,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2), metadataMap(METADATA,SYSTEM,MAP,max,FIX_VALUE),
		VALUE = #max{D,PROVIDER,SOURCE,INFO : connectToMap(k,SYSTEM,MAP,PROVIDER,SOURCE,INFO), metadataStream(k,METADATA,SYSTEM,PROVIDER,SOURCE,INFO,D)}, VALUE!=#infimum.


% checking consistency of ontology
%:- hasRepresentation(X,_), not scope(X), not valueScope(X).
%:- hasRepresentation(_,Y), not representation(Y).
%:- hasValueScope(X,_,_,_), not representation(X).
%:- hasValueScope(_,Y,_,_), not valueScope(Y).
%:- hasScope(X,_), not entityType(X).
%:- hasScope(_,Y), not scope(Y).
%:- hasUnit(X,_), not valueScope(X).
%:- hasUnit(_,Y), not unit(Y).
%:- interRepresentationOperation(X,_), not representation(X).
%:- interRepresentationOperation(_,Y), not representation(Y).

%:- entity(_,Y), not entityType(Y).
%:- information(X,_,_), not entity(X,_).
%:- information(X,Y,_), not scope(X,Y), not valueScope(Y).
%:- information(_,Y,Z), not hasRepresentation(Y,Z).

% checking
% soft optimization contraint instead of a hard constraint in order to enable partial solitions
:~ requiredStream(SYSTEM,INFO), not stream(k,SYSTEM,_,_,INFO), query(k). [1@5,requiredStream]

% checking QoS parameters of the required streams
success(k,SYSTEM,SOURCE,INFO) :- requiredStream(SYSTEM,INFO), stream(k,SYSTEM,PROVIDER,SOURCE,INFO), requiredDelay(SYSTEM,INFO,REQUIRED_DELAY), requiredAccuracy(SYSTEM,INFO,REQUIRED_ACCURACY), 
		streamDelay(k,SYSTEM,PROVIDER,SOURCE,INFO,DELAY), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,INFO,ACCURACY), REQUIRED_DELAY > DELAY, REQUIRED_ACCURACY < ACCURACY, query(k).
:~ requiredStream(SYSTEM,INFO), not success(k,SYSTEM,_,INFO), query(k). [1@4,requiredStream]

:~ requiredMap(SYSTEM,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2), not map(k,SYSTEM,_,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2), query(k). [1@4,requiredMap]
%:~ requiredStream(SYSTEM,INFO,REQUIRED_DELAY,REQUIRED_ACCURACY), stream(k,SYSTEM,SOURCE,INFO), streamDelay(k,SYSTEM,SOURCE,INFO,DELAY), streamAccuracy(k,SYSTEM,SOURCE,INFO,ACCURACY), 
%		REQUIRED_ACCURACY < ACCURACY, query(k). [1@4,requiredStream]
%:~ requiredStream(SYSTEM,INFO,REQUIRED_DELAY,REQUIRED_ACCURACY), stream(k,SYSTEM,SOURCE,INFO), streamDelay(k,SYSTEM,SOURCE,INFO,DELAY), streamAccuracy(k,SYSTEM,SOURCE,INFO,ACCURACY), 
%		REQUIRED_DELAY > DELAY, query(k). [1@4,requiredStream]


% checking required streams
:~ requiredStreams(SYSTEM,ENTITY_TYPE,SCOPE,REP,REQ_DELAY,REQ_ACCURACY,MIN,MAX), 
		COUNT = #count{ENTITY,SCOPE,REP : stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2)), entity(ENTITY,ENTITY_TYPE), 
		streamDelay(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),DELAY), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),ACCURACY), 
		DELAY < REQ_DELAY, ACCURACY > REQ_ACCURACY }, COUNT < MIN, query(k).[1@5,requiredStream]
:~ requiredStreams(SYSTEM,ENTITY_TYPE,SCOPE,REP,REQ_DELAY,REQ_ACCURACY,MIN,MAX), 
		COUNT = #count{ENTITY,SCOPE,REP : stream(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2)), entity(ENTITY,ENTITY_TYPE), 
		streamDelay(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),DELAY), streamAccuracy(k,SYSTEM,PROVIDER,SOURCE,information(ENTITY,SCOPE,REP,ENTITY2),ACCURACY), 
		DELAY < REQ_DELAY, ACCURACY > REQ_ACCURACY }, COUNT > MAX, query(k).[1@5,requiredStream]


% checking point budget of the systems
:- system(SYSTEM,POINTS), SUM_STREAMS = #sum{COST,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO : streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,_,COST)}, 
		SUM_NODES = #sum{COST,SYSTEM,NODE,ENTITY : node(k,SYSTEM,NODE,ENTITY), nodeCost(SYSTEM,NODE,COST)}, (SUM_NODES + SUM_STREAMS) > POINTS, query(k).

% optimize
%% transfer cost
#minimize { TRANSFER_COST@priorityTransferCost,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO : streamTransfer(k,SYSTEM_SOURCE,PROVIDER,SYSTEM_TARGET,SOURCE,INFO,_,TRANSFER_COST), query(k) }.

%% stream QoS
#minimize { DELAY@priorityDelay,SYSTEM,SOURCE,INFO : requiredStream(SYSTEM,INFO), streamDelay(k,SYSTEM,PROVIDER,SOURCE,INFO,DELAY), query(k) }.
accuracy(k,SYSTEM,INFO,ACCURACY) :- requiredStream(SYSTEM,INFO), ACCURACY = #max{ACC,SYSTEM,INFO : requiredStream(SYSTEM,INFO), streamAccuracy(k,SYSTEM,_,_,INFO,ACC)}, query(k).
#maximize { ACCURACY@priorityAccuracy,SYSTEM,INFO : accuracy(k,SYSTEM,INFO,ACCURACY) }.

%#minimize { DELAY@priorityDelay,SYSTEM,SOURCE,INFO : success(k,SYSTEM,SOURCE,INFO), streamDelay(k,SYSTEM,SOURCE,INFO,DELAY), query(k) }.
%#maximize { ACCURACY@priorityAccuracy,SYSTEM,SOURCE,INFO : success(k,SYSTEM,SOURCE,INFO), streamAccuracy(k,SYSTEM,SOURCE,INFO,ACCURACY), query(k) }.

% map QoS
#minimize { DELAY@priorityDelay,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2 : requiredMap(SYSTEM,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2), 
		metadataMap(k,delay,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,DELAY), query(k) }.
#maximize { VALUE@priorityAccuracy,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2 : requiredMap(SYSTEM,ENTITY_TYPE,SCOPE,REPRESENTATION,ENTITY2), 
		metadataMap(k,accuracy,SYSTEM,MAP,ENTITY_TYPE,SCOPE,REP,ENTITY2,VALUE), query(k) }.

%% cost
sumCost(k,SUM_NODES + SUM_TRANSLATIONS + SUM_EXTRACTIONS) :- 
		%SUM_STREAMS = #sum{COST,SYSTEM,SOURCE,INFO : streamCost(k,SYSTEM,SOURCE,INFO,COST)}, 
		SUM_NODES = #sum{COST,SYSTEM,NODE,ENTITY : node(k,SYSTEM,NODE,ENTITY), nodeCost(SYSTEM,NODE,COST)}, 
		SUM_TRANSLATIONS = #sum{COST,SYSTEM,IRO,ENTITY,SCOPE : informationTranslation(k,SYSTEM,IRO,ENTITY,SCOPE), iroCost(SYSTEM,IRO,COST)},
		SUM_EXTRACTIONS = #count{SYSTEM,PROVIDER,SOURCE,SCOPE,REP1,INFO :  informationExtrection(k,SYSTEM,PROVIDER,SOURCE,SCOPE,REP1,INFO)}, query(k).
#minimize { COST@priorityCost : sumCost(k,COST), query(k) }.


%--------------------------------------------------------------------------------------
#program entity(entity,entityType).

entity(entity,entityType).


%--------------------------------------------------------------------------------------
#program sourceNode(node,system,source,entity,scope,rep,entity2,delay,accuracy,cost).

#external sourceNode(system,node,entity).

output(system,node,scope,rep).

nodeAccuracyFix(system,node,accuracy).
nodeDelayFix(system,node,delay).
nodeCost(system,node,cost).


%--------------------------------------------------------------------------------------
#program map(system,map,entityType,scope,rep,entity2,minCount,maxCount,delayType,delayFix,accuracyType,accuracyFix).

#external mapTemplate(system,map,entityType,scope,rep,entity2). 
inputMap(system,map,minCount,maxCount).
%mapDelay(system,map,delayValue).
%mapAccuracy(system,map,accMod).
metadataMap(delay,system,map,delayType,delayFix).
metadataMap(accuracy,system,map,accuracyType,accuracyFix).


%--------------------------------------------------------------------------------------
#program requiredStream(system,information,delay,accuracy).

#external requiredStream(system,information).
requiredAccuracy(system,information,accuracy).
requiredDelay(system,information,delay).


%--------------------------------------------------------------------------------------
#program requiredMap(system,entity_type,scope,representation,entity2).

#external requiredMap(system,entity_type,scope,representation,entity2).


%--------------------------------------------------------------------------------------
#program system(name,points).

#external system(name,points).


%--------------------------------------------------------------------------------------
#program transfer(system1,system2,delay,cost).

#external transfer(system1,system2,delay,cost).
transfer(system2,system1,delay,cost) :- transfer(system1,system2,delay,cost).









