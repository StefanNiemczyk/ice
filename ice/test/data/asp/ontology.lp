entityType(robot).

scope(position).

representation(coords).
representation(wgs84).

representation(floatRep).

valueScope(x).
valueScope(y).
valueScope(z).

valueScope(lat).
valueScope(lon).
valueScope(alt).

unit(degree).
unit(meter).


hasScope(robot,position).

hasRepresentation(position,coords).
hasRepresentation(position,wgs84).

hasValueScope(coords,x,1,1).
hasValueScope(coords,y,1,1).
hasValueScope(coords,z,1,1).

hasValueScope(wgs84,lat,1,1).
hasValueScope(wgs84,lon,1,1).
hasValueScope(wgs84,alt,1,1).

hasRepresentation(x,floatRep).
hasRepresentation(y,floatRep).
hasRepresentation(z,floatRep).

hasRepresentation(lat,floatRep).
hasRepresentation(lon,floatRep).
hasRepresentation(alt,floatRep).

hasUnit(lat,degree).
hasUnit(lon,degree).
hasUnit(alt,meter).
