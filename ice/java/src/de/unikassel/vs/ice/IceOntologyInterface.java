package de.unikassel.vs.ice;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassAssertionAxiom;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLIndividual;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyCreationException;
import org.semanticweb.owlapi.model.OWLOntologyIRIMapper;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.reasoner.OWLReasoner;
import org.semanticweb.owlapi.reasoner.OWLReasonerFactory;
import org.semanticweb.owlapi.reasoner.structural.StructuralReasonerFactory;
import org.semanticweb.owlapi.util.AutoIRIMapper;

public class IceOntologyInterface {

	private static final String ICE_IRI = "http://www.semanticweb.org/sni/ontologies/2013/7/Ice";
	private static final String ICE_IRI_PREFIX = ICE_IRI + "#";

	private OWLOntologyManager manager;
	private OWLOntology mainOntology;
	private OWLReasonerFactory reasonerFactory;
	private OWLReasoner reasoner;
	private OWLDataFactory dataFactory;

	private OWLClass systemOWLClass;

	private List<String> ontologyIries;
	private List<SystemContainer> systems;

	private String mainIRI;
	private String mainIRIPrefix;

	public IceOntologyInterface() {
		this.manager = OWLManager.createOWLOntologyManager();
		// OWLReasonerFactory reasonerFactory = new Reasoner.ReasonerFactory();
		this.reasonerFactory = new StructuralReasonerFactory();
		this.ontologyIries = new ArrayList<String>();
		this.systems = new ArrayList<SystemContainer>();
	}

	public void addIRIMapper(final String p_path) {
		OWLOntologyIRIMapper aim = new AutoIRIMapper(new File(p_path), true);
		this.manager.getIRIMappers().add(aim);
	}

	public boolean loadOntologies() throws OWLOntologyCreationException {

		if (this.ontologyIries.size() == 0)
			return false;
		// TODO merging

		this.mainIRI = this.ontologyIries.get(0);
		this.mainIRIPrefix = this.mainIRI + "#";
		IRI iri = IRI.create(this.mainIRI);
		this.mainOntology = this.manager.loadOntology(iri);

		// Fetching ontology things: class, properties, usw.
		this.systemOWLClass = this.dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "System"));

		this.reasoner = this.reasonerFactory.createReasoner(this.mainOntology);

		return true;
	}

	public boolean isConsistent() {
		return this.reasoner.isConsistent();
	}

	public boolean addSystem(final String p_systemName) {
		// check existing systems
		for (SystemContainer sys : this.systems) {
			if (sys.getName().equals(p_systemName))
				return false;
		}

		SystemContainer system = new SystemContainer(p_systemName);

		OWLIndividual systemInd = dataFactory.getOWLNamedIndividual(IRI.create(this.mainIRIPrefix + p_systemName));
		OWLClassAssertionAxiom ax = dataFactory.getOWLClassAssertionAxiom(this.systemOWLClass, systemInd);
		// Add this axiom to our ontology - with a convenience method
		manager.addAxiom(this.mainOntology, ax);

		system.setIndividual(systemInd);

		this.systems.add(system);

		return true;
	}

	public boolean addOntologyIRI(final String p_iri) {
		return this.ontologyIries.add(p_iri);
	}

	public boolean removeOntologyIRI(final String p_iri) {
		return this.ontologyIries.remove(p_iri);
	}
}
