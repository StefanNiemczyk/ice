package de.unikassel.vs.ice;

public class ICEOntologyInterface {

	private String ontologyPath;

	public String getOntologyPath() {
		return ontologyPath;
	}

	public void setOntologyPath(String ontologyPath) {
		this.ontologyPath = ontologyPath;
	}

	public ICEOntologyInterface(String p_path) {
		this.ontologyPath = p_path;
	}

	public static void test() {
		System.out.println("tut");
	}

	public static void main(String[] args) {
		test();
	}

}
