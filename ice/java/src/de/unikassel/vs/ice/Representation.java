package de.unikassel.vs.ice;

public final class Representation implements Comparable<Representation> {
	private static final String DELIM = ";";
	public Representation parent;
	public String name;

    public Representation(String name, Representation parent) {
        this.name = name;
        this.parent = parent;
    }

    @Override
    public int compareTo(Representation o) {
        // TODO: Check order
        return name.compareTo(o.name);
    }
    
    @Override
    public String toString() {
    	return name + DELIM + parent;
    }
}