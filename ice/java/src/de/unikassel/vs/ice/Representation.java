package de.unikassel.vs.ice;

public final class Representation implements Comparable<Representation> {
	public static final char DELIM = ';';
	public static final String DELIM_STR = DELIM + "";
	public Representation parent;
	public String name;

	public Representation(final String name, final Representation parent) {
		this.name = name;
		this.parent = parent;
	}

	@Override
	public int compareTo(final Representation o) {
		// TODO: Check order
		return name.compareTo(o.name);
	}

	@Override
	public String toString() {
		return name + DELIM + parent;
	}
}