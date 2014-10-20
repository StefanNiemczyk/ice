package de.unikassel.vs.ice;

import java.io.File;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.model.AddAxiom;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassAssertionAxiom;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLDataProperty;
import org.semanticweb.owlapi.model.OWLDataPropertyAssertionAxiom;
import org.semanticweb.owlapi.model.OWLIndividual;
import org.semanticweb.owlapi.model.OWLNamedIndividual;
import org.semanticweb.owlapi.model.OWLObjectProperty;
import org.semanticweb.owlapi.model.OWLObjectPropertyAssertionAxiom;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyChange;
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
	private OWLClass nodeOWLClass;
	private OWLClass metadataOWLClass;
	private OWLClass entityTypeOWLClass;
	private OWLClass aspMetadataGroundingOWLClass;
	private OWLObjectProperty hasSystemOWLProperty;
	private OWLObjectProperty isSystemOfOWLProperty;
	private OWLObjectProperty hasMetadataOWLProperty;
	private OWLObjectProperty isMetadataOfOWLProperty;
	private OWLObjectProperty hasGroundingOWLProperty;
	private OWLObjectProperty isGroundingOfOWLProperty;
	private OWLDataProperty hasMetadataValueOWLDataProperty;

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
		this.nodeOWLClass = this.dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "Node"));
		this.metadataOWLClass = this.dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "Metadata"));
		this.entityTypeOWLClass = this.dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "EntityType"));
		this.aspMetadataGroundingOWLClass = this.dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX
				+ "ASPMetadataGrounding"));
		this.hasSystemOWLProperty = this.dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasSystem"));
		this.isSystemOfOWLProperty = this.dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "isSystemOf"));
		this.hasMetadataOWLProperty = this.dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasMetadata"));
		this.isMetadataOfOWLProperty = this.dataFactory.getOWLObjectProperty(IRI
				.create(ICE_IRI_PREFIX + "isMetadataOf"));
		this.hasGroundingOWLProperty = this.dataFactory.getOWLObjectProperty(IRI
				.create(ICE_IRI_PREFIX + "hasGrounding"));
		this.isGroundingOfOWLProperty = this.dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX
				+ "isGroundingOf"));
		this.hasMetadataValueOWLDataProperty = this.dataFactory.getOWLDataProperty(IRI.create(ICE_IRI_PREFIX
				+ "hasMetadataValue"));

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

	public boolean addNode(final String p_node, final String p_nodeClass, final String p_system, String[] p_metadatas,
			int[] p_metadataValues, String[] p_metadataGroundings) {
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
		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();
		OWLIndividual nodeInd = this.dataFactory.getOWLNamedIndividual(nodeIRI);

		OWLClass nodeCls = this.findOWLClass(this.nodeOWLClass, p_nodeClass);

		if (nodeCls == null) {
			System.out.println(String.format("Unknown node class %s for node %s in system %s, node not created.",
					nodeCls, p_node, p_system));
			return false;
		}

		OWLClassAssertionAxiom ax = this.dataFactory.getOWLClassAssertionAxiom(nodeCls, nodeInd);
		AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, ax);
		changes.add(addAxiomChange);

		OWLObjectPropertyAssertionAxiom assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(
				hasSystemOWLProperty, nodeInd, system.getIndividual());
		addAxiomChange = new AddAxiom(this.mainOntology, assertion);
		changes.add(addAxiomChange);

		// ---
		// TODO remove when new hermit version available
		assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.isSystemOfOWLProperty,
				system.getIndividual(), nodeInd);
		addAxiomChange = new AddAxiom(this.mainOntology, assertion);
		changes.add(addAxiomChange);
		// ---

		// add metadata
		for (int i = 0; i < p_metadatas.length; ++i) {
			String metadata = p_metadatas[i];
			String grounding = p_metadataGroundings[i];
			int value = p_metadataValues[i];

			OWLClass metadataCls = this.findOWLClass(this.metadataOWLClass, metadata);

			if (metadataCls == null) {
				System.out.println(String.format("Unknown Metadata %s for node %s in system %s, node not created.",
						metadata, p_node, p_system));
				return false;
			}

			OWLIndividual metadataInd = this.dataFactory.getOWLNamedIndividual(IRI.create(this.mainIRIPrefix + p_node
					+ metadata));

			ax = this.dataFactory.getOWLClassAssertionAxiom(metadataCls, metadataInd);
			addAxiomChange = new AddAxiom(this.mainOntology, ax);
			changes.add(addAxiomChange);

			assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.hasMetadataOWLProperty, nodeInd,
					metadataInd);
			addAxiomChange = new AddAxiom(this.mainOntology, assertion);
			changes.add(addAxiomChange);

			// ---
			// TODO remove when new hermit version available
			assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.isMetadataOfOWLProperty, metadataInd,
					nodeInd);
			addAxiomChange = new AddAxiom(this.mainOntology, assertion);
			changes.add(addAxiomChange);
			// ---

			OWLDataPropertyAssertionAxiom dataAssertion = this.dataFactory.getOWLDataPropertyAssertionAxiom(
					this.hasMetadataValueOWLDataProperty, metadataInd, value);
			addAxiomChange = new AddAxiom(this.mainOntology, dataAssertion);
			changes.add(addAxiomChange);

			// set grounding
			OWLIndividual metadataGrounding = this.findOWLIndividual(this.aspMetadataGroundingOWLClass, grounding);

			if (metadataGrounding == null) {
				System.out.println(String.format(
						"Unknown Metadata grounding %s of grounding %s for node %s in system %s, node not created.",
						grounding, metadata, p_node, p_system));
				return false;
			}

			assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.hasGroundingOWLProperty, metadataInd,
					metadataGrounding);
			addAxiomChange = new AddAxiom(this.mainOntology, assertion);
			changes.add(addAxiomChange);

			// ---
			// TODO remove when new hermit version available
			assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.isGroundingOfOWLProperty,
					metadataGrounding, metadataInd);
			addAxiomChange = new AddAxiom(this.mainOntology, assertion);
			changes.add(addAxiomChange);
			// ---
		}

		// apply changes
		for (OWLOntologyChange change : changes) {
			this.manager.applyChange(change);
		}

		return true;
	}

	public boolean addOntologyIRI(final String p_iri) {
		return this.ontologyIries.add(p_iri);
	}

	public boolean removeOntologyIRI(final String p_iri) {
		return this.ontologyIries.remove(p_iri);
	}

	public String readInformationStructureAsASP() {
		Set<OWLOntology> onts = new HashSet<OWLOntology>();
		onts.add(this.mainOntology);
		onts.addAll(this.imports);

		InfoStructureVisitor isv = new InfoStructureVisitor(onts, this.reasoner, this.dataFactory);

		Set<OWLClass> classes = this.reasoner.getSubClasses(this.entityTypeOWLClass, true).getFlattened();

		for (OWLClass cls : classes) {
			cls.accept(isv);
		}

		return isv.toString();
	}

	public String readNodesAndIROsAsASP() {
		Set<OWLOntology> onts = new HashSet<OWLOntology>();
		onts.add(this.mainOntology);
		onts.addAll(this.imports);

		NodeIROVisitor niv = new NodeIROVisitor(onts, this.reasoner, this.dataFactory);

		niv.start();

		return niv.toString();
	}

	private OWLIndividual findOWLIndividual(final OWLClass p_class, final String p_name) {
		Set<OWLClass> subs = this.getAllLeafs(p_class);

		for (OWLClass metadata : subs) {
			Set<OWLNamedIndividual> inds = this.reasoner.getInstances(metadata, true).getFlattened();

			for (OWLNamedIndividual ind : inds) {
				if (ind.getIRI().getShortForm().equals(p_name))
					return ind;
			}
		}

		return null;
	}

	private OWLClass findOWLClass(final OWLClass p_class, final String p_name) {
		Set<OWLClass> subs = this.getAllLeafs(p_class);

		for (OWLClass metadata : subs) {
			if (metadata.getIRI().getShortForm().equals(p_name))
				return metadata;
		}

		return null;
	}

	private Set<OWLClass> getAllLeafs(final OWLClass p_start) {
		Set<OWLClass> subClasses = this.reasoner.getSubClasses(p_start, true).getFlattened();
		Set<OWLClass> results = new HashSet<OWLClass>();

		for (OWLClass c : subClasses) {
			if (c.isOWLNothing()) {
				results.add(p_start);
			}

			results.addAll(this.getAllLeafs(c));
		}

		return results;
	}
}
