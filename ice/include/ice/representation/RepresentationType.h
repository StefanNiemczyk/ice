#ifndef REPRESENTATION_TYPE_H
#define REPRESENTATION_TYPE_H

namespace ice {

enum RepresentationType {
  NonNumericalRepresentation,
    BooleanRep, ByteRep, StringRep, UnsignedByteRep,

  NumericalRepresentation,
    FloatNumericalRepresentation,
      DoubleRep, FloatRep,
    IntegerNumericalRepresentation,
      IntegerRep, LongRep, ShortRep,
      UnsignedIntegerRep, UnsignedLongRep, UnsignedShortRep,

  CompositeRepresentation,
    MovementRepresentation,
      DefaultMovementRep,
    OrientationRepresentation,
      EulerAnglesRep,
      RollPitchYawRep,
    PositionRepresentation,
      AddressRep,
      CoordinatPositionRep,
      RelativeCoordinatePositionRep,
      WGS84Rep,
    TimestampRepresentation,
      UnixTimeRep,
    UnknownType,
};

}
#endif // REPRESENTATION_TYPE_H
