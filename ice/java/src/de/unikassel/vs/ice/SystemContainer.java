package de.unikassel.vs.ice;

import org.semanticweb.owlapi.model.OWLIndividual;

public class SystemContainer {
	private String name;
	private OWLIndividual individual;

	public SystemContainer(final String p_name) {
		this.name = p_name;
	}

	public String getName() {
		return this.name;
	}

	public OWLIndividual getIndividual() {
		return individual;
	}

	public void setIndividual(OWLIndividual individual) {
		this.individual = individual;
	}
}
