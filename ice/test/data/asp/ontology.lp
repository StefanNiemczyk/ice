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

hasRelatedDimension(relCoords,position,coords).

hasDimension(relCoords,x,floatRep).
hasDimension(relCoords,y,floatRep).
hasDimension(relCoords,z,floatRep).

hasDimension(coords,x,floatRep).
hasDimension(coords,y,floatRep).
hasDimension(coords,z,floatRep).

hasDimension(coords2D,x,floatRep).
hasDimension(coords2D,y,floatRep).

hasDimension(wgs84,lat,floatRep).
hasDimension(wgs84,lon,floatRep).
hasDimension(wgs84,alt,floatRep).

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








