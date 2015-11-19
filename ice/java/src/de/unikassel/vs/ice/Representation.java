package de.unikassel.vs.ice;

public enum Representation {
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
}
