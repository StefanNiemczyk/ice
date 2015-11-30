package de.unikassel.vs.ice;

public enum LogLevel {
    Error(0),
    Warning(1),
    Debug(2),
    Info(3);

    private final int value;

    LogLevel(final int newValue) {
        value = newValue;
    }

    public int getValue() { return value; }
}