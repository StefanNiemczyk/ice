entityType(robot).

scope(position).

representation(relCoords).
representation(coords).
representation(coords2D).
representation(wgs84).

representation(floatRep).

valueScope(x).
valueScope(y).
valueScope(z).

valueScopeDefault(z,0).

valueScope(lat).
valueScope(lon).
valueScope(alt).

unit(degree).
unit(meter).


hasScope(robot,position).

hasRepresentation(position,relCoords).
hasRepresentation(position,coords).
hasRepresentation(position,coords2D).
hasRepresentation(position,wgs84).

hasRelatedDimension(relCoords,position,coords,1,1).

hasDimension(relCoords,x,1,1).
hasDimension(relCoords,y,1,1).
hasDimension(relCoords,z,1,1).

hasDimension(coords,x,floatRep,1,1).
hasDimension(coords,y,floatRep,1,1).
hasDimension(coords,z,floatRep,1,1).

hasDimension(coords2D,x,floatRep,1,1).
hasDimension(coords2D,y,floatRep,1,1).

hasDimension(wgs84,lat,floatRep,1,1).
hasDimension(wgs84,lon,floatRep,1,1).
hasDimension(wgs84,alt,floatRep,1,1).

hasRepresentation(x,floatRep).
hasRepresentation(y,floatRep).
hasRepresentation(z,floatRep).

hasRepresentation(lat,floatRep).
hasRepresentation(lon,floatRep).
hasRepresentation(alt,floatRep).

hasUnit(lat,degree).
hasUnit(lon,degree).
hasUnit(alt,meter).




% velocity scope
scope(velocity).

representation(vel1).
representation(vel2).

hasRepresentation(velocity,vel1).
hasRepresentation(velocity,vel2).

scope(movement).

representation(mov1).
representation(mov2).

hasRepresentation(movement,mov1).
hasRepresentation(movement,mov2).

hasDimension(mov1,position,coords,1,1).
hasDimension(mov1,velocity,vel1,1,1).

hasDimension(mov2,position,coords,1,1).
hasDimension(mov2,velocity,vel2,1,1).








