%--------------------------------------------------------------------------------------
#program entity(entity,entityType).

entity(entity,entityType).


%--------------------------------------------------------------------------------------
#program sourceNode(node,system,source,entity,scope,rep,entity2,delayFix,accuracyFix,costValue).

#external sourceNode(system,node,entity).

output(system,node,scope,rep,none).

metadataOutput(accuracy,system,node,fix,accuracyFix,0).
metadataOutput(delay,system,node,fix,delayFix,0).
metadataProcessing(cost,system,node,costValue).


%--------------------------------------------------------------------------------------
#program requiredStream(system,information,delayValue,accuracyValue).

#external requiredStream(system,information).
requiredMetadata(accuracy,system,information,accuracyValue).
requiredMetadata(delay,system,information,delayValue).


%--------------------------------------------------------------------------------------
#program requiredSet(system,informationType).

#external requiredSet(system,informationType).


%--------------------------------------------------------------------------------------
#program transfer(system1,system2,delayValue,costValue).

#external transfer(system1,system2).
transfer(system2,system1) :- transfer(system1,system2).

metadataProcessing(cost,system1,system2,costValue).
metadataOutput(delay,system1,system2,delayValue).

metadataProcessing(cost,system2,system1,costValue).
metadataOutput(delay,system2,system1,delayValue).


%--------------------------------------------------------------------------------------
#program system(name).

#external system(name).

%--------------------------------------------------------------------------------------
#program system(name,points).

#external system(name).
systemPoints(name,points) :- system(name).