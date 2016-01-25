package de.unikassel.vs.ice;

public enum LogLevel {
	Disabled(0), Error(1), Warn(2), Debug(3), Info(4);

	private final int value;

	LogLevel(final int newValue) {
		value = newValue;
	}

	public int getValue() {
		return value;
	}
}