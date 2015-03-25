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

valueScope(lat).
valueScope(lon).
valueScope(alt).

unit(degree).
unit(meter).


hasScope(robot,position).

hasRepresentation(position,relCoords).
hasRepresentation(position,coords).
hasRepresentation(position,wgs84).

hasDimension(relCoords,position,coords,1,1).

hasDimension(relCoords,x,1,1).
hasDimension(relCoords,y,1,1).
hasDimension(relCoords,z,1,1).

hasDimension(coords,x,1,1).
hasDimension(coords,y,1,1).
hasDimension(coords,z,1,1).

hasDimension(coords2D,x,1,1).
hasDimension(coords2D,y,1,1).

hasDimension(wgs84,lat,1,1).
hasDimension(wgs84,lon,1,1).
hasDimension(wgs84,alt,1,1).

hasRepresentation(x,floatRep).
hasRepresentation(y,floatRep).
hasRepresentation(z,floatRep).

hasRepresentation(lat,floatRep).
hasRepresentation(lon,floatRep).
hasRepresentation(alt,floatRep).

hasUnit(lat,degree).
hasUnit(lon,degree).
hasUnit(alt,meter).
