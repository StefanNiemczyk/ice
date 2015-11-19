package de.unikassel.vs.ice;

public final class RepresentationIndividual implements Comparable<RepresentationIndividual> {

	public static final char DELIM = ';';
	public static final String DELIM_STR = "" + DELIM;
	public static final String ESCAPED_DELIM_STR = "\\" + DELIM;

	private Representation representation;
	private String dataString;

	public RepresentationIndividual(final String representationString, final String dataString) {
		this.representation = Representation.valueOf(representationString);
		this.dataString = dataString;
	}

	public Representation getRepresentation() {
		return representation;
	}

	public String getDataStr() {
		return dataString;
	}

	@Override
	public String toString() {
		final int repNum = representation.ordinal();
		final String dataStr = dataString.replace(DELIM_STR, ESCAPED_DELIM_STR);
		return repNum + DELIM_STR + dataStr;
	}

	@Override
	public int compareTo(final RepresentationIndividual o) {
		return representation.ordinal() - o.getRepresentation().ordinal();
	}

}