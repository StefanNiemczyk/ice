package de.unikassel.vs.ice;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.model.AddAxiom;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassAssertionAxiom;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLIndividual;
import org.semanticweb.owlapi.model.OWLObjectProperty;
import org.semanticweb.owlapi.model.OWLObjectPropertyAssertionAxiom;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyCreationException;
import org.semanticweb.owlapi.model.OWLOntologyIRIMapper;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.reasoner.OWLReasoner;
import org.semanticweb.owlapi.reasoner.OWLReasonerFactory;
import org.semanticweb.owlapi.reasoner.structural.StructuralReasonerFactory;
import org.semanticweb.owlapi.util.AutoIRIMapper;

public class IceOntologyInterface {

	public static final String ICE_IRI = "http://www.semanticweb.org/sni/ontologies/2013/7/Ice";
	public static final String ICE_IRI_PREFIX = ICE_IRI + "#";

	private OWLOntologyManager manager;
	private OWLOntology mainOntology;
	private Set<OWLOntology> imports;
	private OWLReasonerFactory reasonerFactory;
	private OWLReasoner reasoner;
	private OWLDataFactory dataFactory;

	private OWLClass systemOWLClass;
	private OWLClass entityTypeOWLClass;
	private OWLObjectProperty hasSystemOWLProperty;
	private OWLObjectProperty isSystemOfOWLProperty;

	private List<String> ontologyIries;
	private List<SystemContainer> systems;

	private String mainIRI;
	private String mainIRIPrefix;

	public IceOntologyInterface() {
		this.manager = OWLManager.createOWLOntologyManager();
		this.dataFactory = manager.getOWLDataFactory();
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
		this.imports = this.manager.getImports(this.mainOntology);

		// Fetching ontology things: class, properties, usw.
		this.systemOWLClass = this.dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "System"));
		this.entityTypeOWLClass = this.dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "EntityType"));
		this.hasSystemOWLProperty = this.dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasSystem"));
		this.isSystemOfOWLProperty = this.dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "isSystemOf"));

		this.reasoner = this.reasonerFactory.createReasoner(this.mainOntology);
		this.reasoner.precomputeInferences();

		return true;
	}

	public boolean isConsistent() {
		return this.reasoner.isConsistent();
	}

	public boolean addSystem(final String p_systemName) {
		// check existing systems
		if (this.mainOntology.containsIndividualInSignature(IRI.create(this.mainIRIPrefix + p_systemName)))
			return false;

		// for (SystemContainer sys : this.systems) {
		// if (sys.getName().equals(p_systemName))
		// return false;
		// }

		SystemContainer system = new SystemContainer(p_systemName);

		OWLIndividual systemInd = dataFactory.getOWLNamedIndividual(IRI.create(this.mainIRIPrefix + p_systemName));
		OWLClassAssertionAxiom ax = dataFactory.getOWLClassAssertionAxiom(this.systemOWLClass, systemInd);
		// Add this axiom to our ontology - with a convenience method
		this.manager.addAxiom(this.mainOntology, ax);

		system.setIndividual(systemInd);

		this.systems.add(system);

		return true;
	}

	public boolean addNode(final String p_node, final String p_system, String[] p_metadatas, int[] p_metadataValues) {
		// check if node exists
		IRI nodeIRI = IRI.create(this.mainIRIPrefix + p_node);
		if (this.mainOntology.containsIndividualInSignature(nodeIRI))
			return false;

		SystemContainer system = null;

		// Get System
		for (SystemContainer sys : this.systems) {
			if (sys.getName().equals(p_system)) {
				system = sys;
			}
		}

		if (system == null) {
			SystemContainer sys = new SystemContainer(p_system);

			OWLIndividual systemInd = this.dataFactory.getOWLNamedIndividual(IRI.create(this.mainIRIPrefix + p_system));
			OWLClassAssertionAxiom ax = this.dataFactory.getOWLClassAssertionAxiom(this.systemOWLClass, systemInd);
			// Add this axiom to our ontology - with a convenience method
			this.manager.addAxiom(this.mainOntology, ax);

			sys.setIndividual(systemInd);

			this.systems.add(sys);
			system = sys;
		}

		// create node
		OWLIndividual nodeInd = this.dataFactory.getOWLNamedIndividual(nodeIRI);

		OWLObjectPropertyAssertionAxiom assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(
				hasSystemOWLProperty, nodeInd, system.getIndividual());
		AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, assertion);
		this.manager.applyChange(addAxiomChange);

		// ---
		// TODO remove when new hermit version available
		assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.isSystemOfOWLProperty,
				system.getIndividual(), nodeInd);
		addAxiomChange = new AddAxiom(this.mainOntology, assertion);
		this.manager.applyChange(addAxiomChange);
		// ---

		// add metadata
		for (int i = 0; i < p_metadatas.length; ++i) {
			String metadata = p_metadatas[i];
			int value = p_metadataValues[i];

			// TODO
		}

		return false;
	}

	public boolean addOntologyIRI(final String p_iri) {
		return this.ontologyIries.add(p_iri);
	}

	public boolean removeOntologyIRI(final String p_iri) {
		return this.ontologyIries.remove(p_iri);
	}

	public String readInformationStructureAsASP() {
		InfoStructureVisitor isv = new InfoStructureVisitor(this.imports, this.reasoner, this.dataFactory);

		Set<OWLClass> classes = this.reasoner.getSubClasses(this.entityTypeOWLClass, true).getFlattened();

		for (OWLClass cls : classes) {
			cls.accept(isv);
		}

		return isv.toString();
	}
}
