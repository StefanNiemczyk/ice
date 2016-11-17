package de.unikassel.vs.ice;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.model.AddAxiom;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassAssertionAxiom;
import org.semanticweb.owlapi.model.OWLClassExpression;
import org.semanticweb.owlapi.model.OWLDataFactory;
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
import org.semanticweb.owlapi.model.OWLSubClassOfAxiom;
import org.semanticweb.owlapi.reasoner.OWLReasoner;
import org.semanticweb.owlapi.reasoner.OWLReasonerFactory;
import org.semanticweb.owlapi.reasoner.structural.StructuralReasoner;
import org.semanticweb.owlapi.reasoner.structural.StructuralReasonerFactory;
import org.semanticweb.owlapi.util.AutoIRIMapper;

public class IceOntologyInterface {
	public static SimpleDateFormat DATE_FORMAT = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss,SSS");

	private OWLOntologyManager manager;
	private OWLOntology mainOntology;
	private Set<OWLOntology> imports;
	private OWLReasonerFactory reasonerFactory;
	private OWLReasoner internalReasoner;
	private OWLDataFactory dataFactory;

	private IceIris ii;

	private List<String> ontologyIries;
	private List<String> iriMapping;
	private boolean dirty;

	private int someMinCardinality;
	private int someMaxCardinality;
	private int defaultMinCardinality;
	private int defaultMaxCardinality;

	private LogLevel logLevel = LogLevel.Info;

	private String mainIRI;
	private String mainIRIPrefix;

	Thread memoryMonitor;
	public long[] memoryUsage = new long[3];
	public boolean memoryMonitorState = false;

	public IceOntologyInterface() {
		// System.gc();
		this.manager = OWLManager.createOWLOntologyManager();
		this.dataFactory = manager.getOWLDataFactory();
		// this.reasonerFactory = new Reasoner.ReasonerFactory();
		this.reasonerFactory = new StructuralReasonerFactory();
		this.ontologyIries = new ArrayList<String>();
		this.someMinCardinality = 1;
		this.someMaxCardinality = 1;
		this.defaultMinCardinality = 1;
		this.defaultMaxCardinality = 5;
		this.dirty = true;
		this.iriMapping = new ArrayList<String>();
	}

	public void cleanUp() {
		System.gc();
	}

	public void startMemoryMonitor() {
		if (memoryMonitorState == true)
			return;

		this.memoryMonitorState = true;

		Thread t = new Thread(new MemoryMonitor(this));

		t.start();
	}

	public void stopMemoryMonitor() {
		this.memoryMonitorState = false;
	}

	public void resetMemoryMonitor() {
		memoryUsage = new long[3];
	}

	public long[] getMemoryUsage() {
		return this.memoryUsage;
	}

	public void addIRIMapper(final String p_path) {
		OWLOntologyIRIMapper aim = new AutoIRIMapper(new File(p_path), true);
		this.manager.addIRIMapper(aim);
	}

	public boolean loadOntology(final String p_ontPath) {
		if (p_ontPath == null)
			return false;

		try {
			File ont = new File(p_ontPath);
			this.mainOntology = this.manager.loadOntologyFromOntologyDocument(ont);
			this.mainIRI = this.mainOntology.getOntologyID().getOntologyIRI().toString();
			this.mainIRIPrefix = this.mainIRI + "#";
			this.imports = this.manager.getImports(this.mainOntology);

			// Updating Mapping
			this.updateIriMapping();

			// Fetching ontology things: class, properties, usw.
			this.ii = new IceIris(this.dataFactory);

			this.dirty = true;

			return true;
		} catch (Exception e) {
			logError("Exception during loading the ontology " + e.getMessage());
			return false;
		}
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

		// Updating Mapping
		this.updateIriMapping();

		// Fetching ontology things: class, properties, usw.
		this.ii = new IceIris(this.dataFactory);

		this.dirty = true;

		return true;
	}

	private void updateIriMapping() {
		if (false == this.iriMapping.contains(this.mainIRI))
			this.iriMapping.add(this.mainIRI);

		for (OWLOntology o : this.imports) {
			String iriStr = o.getOntologyID().getOntologyIRI().toString();
			if (false == this.iriMapping.contains(iriStr))
				this.iriMapping.add(iriStr);
		}

		if (false == this.iriMapping.contains("http://www.w3.org/2002/07/owl"))
			this.iriMapping.add("http://www.w3.org/2002/07/owl");
	}

	public boolean saveOntology(final String p_saveTo) {
		try {
			File file = new File(p_saveTo);
			manager.saveOntology(this.mainOntology, IRI.create(file.toURI()));
			return true;
		} catch (Exception e) {
			logError("Exception during saving the ontology " + e.getMessage());
			return false;
		}
	}

	public String[][] getOntologyIDs() {
		String[][] ids = new String[2][this.imports.size() + 1];

		ids[0][0] = this.mainOntology.getOntologyID().getOntologyIRI().toString();
		if (this.mainOntology.getOntologyID().getVersionIRI() != null) {
			ids[1][0] = this.mainOntology.getOntologyID().getVersionIRI().toString();
		} else {
			ids[1][0] = null;
		}

		int i = 1;
		for (OWLOntology ont : this.imports) {
			ids[0][i] = ont.getOntologyID().getOntologyIRI().toString();
			if (ont.getOntologyID().getVersionIRI() != null) {
				ids[1][i] = ont.getOntologyID().getVersionIRI().toString();
			} else {
				ids[1][i] = null;
			}
			++i;
		}

		return ids;
	}

	public boolean isConsistent() {
		return this.getReasoner().isConsistent();
	}

	public boolean addSystem(final String p_systemName) {
		// check existing systems
		OWLIndividual systemInd = this.findOWLIndividual(this.ii.system, p_systemName, false);
		if (systemInd != null)
			return false;

		systemInd = dataFactory.getOWLNamedIndividual(IRI.create(this.mainIRIPrefix + p_systemName));
		OWLClassAssertionAxiom ax = dataFactory.getOWLClassAssertionAxiom(this.ii.system, systemInd);
		// Add this axiom to our ontology - with a convenience method
		this.manager.addAxiom(this.mainOntology, ax);

		this.dirty = true;

		return true;
	}

	public boolean addNodesToSystem(final String p_system, final String[] p_toAdd) {
		// check existing systems
		OWLIndividual systemInd = this.findOWLIndividual(this.ii.system, p_system, false);
		if (systemInd == null) {
			logWarning(String.format("No system '%s' found, isSystemOf will not be established", p_system));
			return false;
		}

		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();

		for (String toAdd : p_toAdd) {
			OWLIndividual nodeInd = this.findOWLIndividual(this.ii.node, toAdd, false);

			if (nodeInd == null) {
				logWarning(String.format("No node '%s' found, isSystemOf for system '%s' will not be established.",
						toAdd, p_system));
				return false;
			}

			OWLObjectPropertyAssertionAxiom assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(
					this.ii.isSystemOf, systemInd, nodeInd);
			AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, assertion);
			changes.add(addAxiomChange);
		}

		// apply changes
		this.manager.applyChanges(changes);

		this.dirty = true;

		return true;
	}

	public boolean addEntityType(final String p_entityType, final String[] p_entityScopes) {
		IRI entityIRI = IRI.create(this.mainIRIPrefix + p_entityType);
		if (this.mainOntology.containsClassInSignature(entityIRI))
			return false;

		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();

		// create entity scope
		OWLClass entity = this.dataFactory.getOWLClass(entityIRI);
		OWLSubClassOfAxiom axiom = dataFactory.getOWLSubClassOfAxiom(entity, this.ii.entityType);
		AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, axiom);
		changes.add(addAxiomChange);

		// create entity scopes
		for (String entityScope : p_entityScopes) {
			OWLClass es = this.findOWLClass(this.ii.entityScope, entityScope);

			if (es == null) {
				logWarning(String.format("Unknown entity scope class '%s' for entity '%s', entity type not created.",
						entityScope, p_entityType));
				return false;
			}

			OWLClassExpression hasScope = this.dataFactory.getOWLObjectSomeValuesFrom(this.ii.hasScope, es);
			OWLSubClassOfAxiom ax = this.dataFactory.getOWLSubClassOfAxiom(entity, hasScope);
			addAxiomChange = new AddAxiom(this.mainOntology, ax);
			changes.add(addAxiomChange);
		}

		// apply changes
		this.manager.applyChanges(changes);

		this.dirty = true;

		return true;
	}

	public boolean addScopesToEntityType(final String p_entityType, final String[] p_entityScopes) {
		IRI entityIRI = IRI.create(this.mainIRIPrefix + p_entityType);
		if (false == this.mainOntology.containsClassInSignature(entityIRI))
			return false;

		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();

		// create entity scope
		OWLClass entity = this.dataFactory.getOWLClass(entityIRI);

		// create entity scopes
		for (String entityScope : p_entityScopes) {
			OWLClass es = this.findOWLClass(this.ii.entityScope, entityScope);

			if (es == null) {
				logWarning(String.format("Unknown entity scope class '%s' for entity '%s', entity type not created.",
						entityScope, p_entityType));
				return false;
			}

			OWLClassExpression hasScope = this.dataFactory.getOWLObjectSomeValuesFrom(this.ii.hasScope, es);
			OWLSubClassOfAxiom ax = this.dataFactory.getOWLSubClassOfAxiom(entity, hasScope);
			AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, ax);
			changes.add(addAxiomChange);
		}

		// apply changes
		this.manager.applyChanges(changes);

		this.dirty = true;

		return true;
	}

	public boolean addEntityScope(final String p_entityScope, final String[] p_representations) {
		IRI entityScopeIRI = IRI.create(this.mainIRIPrefix + p_entityScope);
		if (this.mainOntology.containsClassInSignature(entityScopeIRI))
			return false;

		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();

		// create entity scope
		OWLClass entityScope = this.dataFactory.getOWLClass(entityScopeIRI);
		OWLSubClassOfAxiom axiom = dataFactory.getOWLSubClassOfAxiom(entityScope, this.ii.entityScope);
		AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, axiom);
		changes.add(addAxiomChange);

		// create representations
		for (String representation : p_representations) {
			OWLClass rep = this.findOWLClass(this.ii.representation, representation);

			if (rep == null) {
				// logWarning(String.format(
				// "Unknown representation class '%s' for entity scope '%s', entity scope not created.",
				// representation, p_entityScope));
				// return false;

				rep = this.dataFactory.getOWLClass(IRI.create(this.mainIRIPrefix + representation));
				axiom = dataFactory.getOWLSubClassOfAxiom(rep, this.ii.compositeRepresentation);
				addAxiomChange = new AddAxiom(this.mainOntology, axiom);
				changes.add(addAxiomChange);
			}

			OWLClassExpression hasRepresentation = this.dataFactory.getOWLObjectSomeValuesFrom(
					this.ii.hasRepresentation, rep);
			OWLSubClassOfAxiom ax = this.dataFactory.getOWLSubClassOfAxiom(entityScope, hasRepresentation);
			addAxiomChange = new AddAxiom(this.mainOntology, ax);
			changes.add(addAxiomChange);
		}

		// apply changes
		this.manager.applyChanges(changes);

		this.dirty = true;

		return true;
	}

	public boolean addValueScope(final String p_superValueScope, final String p_valueScope,
			final String p_representation) {
		IRI valueScopeIRI = IRI.create(this.mainIRIPrefix + p_valueScope);
		if (this.mainOntology.containsClassInSignature(valueScopeIRI))
			return false;

		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();

		// create super value scope
		OWLClass superValueScope = this.findOWLClass(this.ii.valueScope, p_superValueScope);

		if (superValueScope == null) {
			superValueScope = this.dataFactory.getOWLClass(IRI.create(this.mainIRIPrefix + p_superValueScope));
			OWLSubClassOfAxiom axiom = dataFactory.getOWLSubClassOfAxiom(superValueScope, this.ii.valueScope);
			AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, axiom);
			changes.add(addAxiomChange);
		}

		// featch representation
		OWLClass representation = this.findOWLClass(this.ii.representation, p_representation);

		if (representation == null) {
			logWarning(String.format(
					"Unknown representation class '%s' for value scope scope '%s', entity scope not created.",
					p_representation, p_valueScope));
			return false;
		}

		// create value scope
		OWLClass valueScope = this.dataFactory.getOWLClass(valueScopeIRI);
		OWLSubClassOfAxiom axiom = dataFactory.getOWLSubClassOfAxiom(valueScope, superValueScope);
		AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, axiom);
		changes.add(addAxiomChange);

		OWLClassExpression hasRepresentation = this.dataFactory.getOWLObjectSomeValuesFrom(this.ii.hasRepresentation,
				representation);
		OWLSubClassOfAxiom ax = this.dataFactory.getOWLSubClassOfAxiom(valueScope, hasRepresentation);
		addAxiomChange = new AddAxiom(this.mainOntology, ax);
		changes.add(addAxiomChange);

		// apply changes
		for (OWLOntologyChange change : changes) {
			this.manager.applyChange(change);
		}

		this.dirty = true;

		return true;
	}

	public boolean addRepresentation(final String p_superRepresentation, final String p_representation,
			final String[] p_dimensions) {
		IRI representationIRI = IRI.create(this.mainIRIPrefix + p_representation);
		if (this.mainOntology.containsClassInSignature(representationIRI))
			return false;

		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();

		// create super represenation
		OWLClass superRep = this.findOWLClass(this.ii.compositeRepresentation, p_superRepresentation);

		if (superRep == null) {
			superRep = this.dataFactory.getOWLClass(IRI.create(this.mainIRIPrefix + p_superRepresentation));
			OWLSubClassOfAxiom axiom = dataFactory.getOWLSubClassOfAxiom(superRep, this.ii.compositeRepresentation);
			AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, axiom);
			changes.add(addAxiomChange);
		}

		// create representation
		OWLClass rep = this.dataFactory.getOWLClass(representationIRI);
		OWLSubClassOfAxiom axiom = dataFactory.getOWLSubClassOfAxiom(rep, superRep);
		AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, axiom);
		changes.add(addAxiomChange);

		// create dimensions
		for (String dimension : p_dimensions) {
			OWLClass dim = this.findOWLClass(this.ii.valueScope, dimension);

			if (dim == null) {
				logWarning(String.format(
						"Unknown dimension class '%s' for representation '%s', representation not created.", dimension,
						p_representation));
				return false;
			}

			OWLClassExpression hasDimension = this.dataFactory.getOWLObjectSomeValuesFrom(this.ii.hasDimension, dim);
			OWLSubClassOfAxiom ax = this.dataFactory.getOWLSubClassOfAxiom(rep, hasDimension);
			addAxiomChange = new AddAxiom(this.mainOntology, ax);
			changes.add(addAxiomChange);

		}

		// apply changes
		for (OWLOntologyChange change : changes) {
			this.manager.applyChange(change);
		}

		this.dirty = true;

		return true;
	}

	public boolean addDimensionToRep(final String p_representation, final String p_dimension, final String p_entityScope) {
		IRI representationIRI = IRI.create(this.mainIRIPrefix + p_representation);
		if (false == this.mainOntology.containsClassInSignature(representationIRI))
			return false;

		OWLClass rep = this.findOWLClass(this.ii.representation, p_representation);
		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();

		OWLClass dim = this.findOWLClass(this.ii.representation, p_dimension);

		if (dim == null) {
			logWarning(String.format(
					"Unknown dimension class '%s' for representation '%s', representation not created.", p_dimension,
					p_representation));
			return false;
		}

		OWLClass scope = this.findOWLClass(this.ii.entityScope, p_entityScope);

		OWLClassExpression hasRepresentation = this.dataFactory.getOWLObjectSomeValuesFrom(this.ii.hasRepresentation,
				dim);

		OWLClassExpression anonymious = this.dataFactory.getOWLObjectIntersectionOf(scope, hasRepresentation);

		OWLClassExpression hasDimension = this.dataFactory.getOWLObjectSomeValuesFrom(this.ii.hasDimension, anonymious);
		OWLSubClassOfAxiom ax = this.dataFactory.getOWLSubClassOfAxiom(rep, hasDimension);
		AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, ax);
		changes.add(addAxiomChange);

		// apply changes
		for (OWLOntologyChange change : changes) {
			this.manager.applyChange(change);
		}

		this.dirty = true;

		return true;
	}

	public boolean addDimensionToRep(final String p_representation, final String p_dimension) {
		IRI representationIRI = IRI.create(this.mainIRIPrefix + p_representation);
		if (false == this.mainOntology.containsClassInSignature(representationIRI))
			return false;

		OWLClass rep = this.findOWLClass(this.ii.representation, p_representation);
		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();

		OWLClass dim = this.findOWLClass(this.ii.valueScope, p_dimension);

		if (dim == null) {
			logWarning(String.format("Dimension class '%s' to add to representation '%s' not found.", p_dimension,
					p_representation));
			return false;
		}

		OWLClassExpression hasDimension = this.dataFactory.getOWLObjectSomeValuesFrom(this.ii.hasDimension, dim);
		OWLSubClassOfAxiom ax = this.dataFactory.getOWLSubClassOfAxiom(rep, hasDimension);
		AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, ax);
		changes.add(addAxiomChange);

		// apply changes
		for (OWLOntologyChange change : changes) {
			this.manager.applyChange(change);
		}

		this.dirty = true;

		return true;
	}

	public boolean addNamedStream(final String p_stream, final String p_entityScope, final String p_representation) {
		IRI streamIRI = IRI.create(this.mainIRIPrefix + p_stream);
		if (this.mainOntology.containsClassInSignature(streamIRI))
			return false;

		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();

		// featch scope and representation
		OWLClass entityScope = this.findOWLClass(this.ii.entityScope, p_entityScope);
		OWLClass representation = this.findOWLClass(this.ii.representation, p_representation);

		if (entityScope == null || representation == null) {
			logWarning(String.format("Unknown entity scope class '%s' for stream '%s', stream not created.",
					p_entityScope, p_stream));
			return false;
		}

		// create value scope
		OWLClass stream = this.dataFactory.getOWLClass(streamIRI);
		OWLSubClassOfAxiom axiom = dataFactory.getOWLSubClassOfAxiom(stream, this.ii.namedStream);
		AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, axiom);
		changes.add(addAxiomChange);

		OWLClassExpression hasRepresentation = this.dataFactory.getOWLObjectSomeValuesFrom(this.ii.hasRepresentation,
				representation);
		OWLClassExpression scope = this.dataFactory.getOWLObjectIntersectionOf(entityScope, hasRepresentation);
		OWLClassExpression isStreamOf = this.dataFactory.getOWLObjectSomeValuesFrom(this.ii.isStreamOf, scope);
		OWLSubClassOfAxiom ax = this.dataFactory.getOWLSubClassOfAxiom(stream, isStreamOf);
		addAxiomChange = new AddAxiom(this.mainOntology, ax);
		changes.add(addAxiomChange);

		// apply changes
		for (OWLOntologyChange change : changes) {
			this.manager.applyChange(change);
		}

		this.dirty = true;

		return true;
	}

	public boolean addNamedMap(final String p_map, final String p_aboutEntity, final String p_entityScope,
			final String p_representation) {
		IRI mapIRI = IRI.create(this.mainIRIPrefix + p_map);
		if (this.mainOntology.containsClassInSignature(mapIRI))
			return false;

		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();

		// featch scope and representation
		OWLClass entity = this.findOWLClass(this.ii.entityType, p_aboutEntity);
		OWLClass entityScope = this.findOWLClass(this.ii.entityScope, p_entityScope);
		OWLClass representation = this.findOWLClass(this.ii.representation, p_representation);

		if (entity == null || entityScope == null || representation == null) {
			logError(String.format("Unknown entity type class '%s' for map '%s', map not created.", p_aboutEntity,
					p_map));
			return false;
		}

		// create value scope
		OWLClass map = this.dataFactory.getOWLClass(mapIRI);
		OWLSubClassOfAxiom axiom = dataFactory.getOWLSubClassOfAxiom(map, this.ii.namedMap);
		AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, axiom);
		changes.add(addAxiomChange);

		OWLClassExpression hasRepresentation = this.dataFactory.getOWLObjectSomeValuesFrom(this.ii.hasRepresentation,
				representation);
		OWLClassExpression scope = this.dataFactory.getOWLObjectIntersectionOf(entityScope, hasRepresentation);
		OWLClassExpression isStreamOf = this.dataFactory.getOWLObjectSomeValuesFrom(this.ii.isMapOf, scope);
		OWLSubClassOfAxiom ax = this.dataFactory.getOWLSubClassOfAxiom(map, isStreamOf);
		addAxiomChange = new AddAxiom(this.mainOntology, ax);
		changes.add(addAxiomChange);

		OWLClassExpression aboutEntity = this.dataFactory.getOWLObjectSomeValuesFrom(this.ii.aboutEntity, entity);
		ax = this.dataFactory.getOWLSubClassOfAxiom(map, aboutEntity);
		addAxiomChange = new AddAxiom(this.mainOntology, ax);
		changes.add(addAxiomChange);

		// apply changes
		for (OWLOntologyChange change : changes) {
			this.manager.applyChange(change);
		}

		this.dirty = true;

		return true;
	}

	public boolean addRequiredStream(final String p_requirdStream, final String p_namedStreamClass,
			final String p_system, final String p_entity, final String p_relatedEntity) {
		IRI requiredStreamIRI = IRI.create(this.mainIRIPrefix + p_requirdStream);
		if (this.mainOntology.containsIndividualInSignature(requiredStreamIRI))
			return false;

		IRI entityIRI = IRI.create(this.mainIRIPrefix + p_entity);

		if (p_entity == null || p_entity.isEmpty()
				|| false == this.mainOntology.containsIndividualInSignature(entityIRI)) {
			logWarning(String.format("Unknown entity '%s' for required stream '%s', stream not created.", p_entity,
					p_requirdStream));
			return false;
		}

		OWLClass namedStreamClass = this.findOWLClass(this.ii.namedStream, p_namedStreamClass);

		if (namedStreamClass == null) {
			logWarning(String.format("Unknown named stream class '%s' for named stream '%s', stream not created.",
					p_namedStreamClass, p_requirdStream));
			return false;
		}

		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();

		OWLIndividual requiredStream = this.dataFactory.getOWLNamedIndividual(requiredStreamIRI);
		OWLIndividual system = this.dataFactory.getOWLNamedIndividual(IRI.create(this.mainIRIPrefix + p_system));

		OWLClassAssertionAxiom ax = this.dataFactory.getOWLClassAssertionAxiom(namedStreamClass, requiredStream);
		AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, ax);
		changes.add(addAxiomChange);

		OWLObjectPropertyAssertionAxiom assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(
				this.ii.isSystemOf, system, requiredStream);
		addAxiomChange = new AddAxiom(this.mainOntology, assertion);
		changes.add(addAxiomChange);

		ax = this.dataFactory.getOWLClassAssertionAxiom(this.ii.requiredStream, requiredStream);
		addAxiomChange = new AddAxiom(this.mainOntology, ax);
		changes.add(addAxiomChange);

		OWLIndividual entityInd = this.dataFactory.getOWLNamedIndividual(entityIRI);

		assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.ii.aboutEntity, requiredStream, entityInd);
		addAxiomChange = new AddAxiom(this.mainOntology, assertion);
		changes.add(addAxiomChange);

		if (p_relatedEntity != null && false == p_relatedEntity.isEmpty()) {
			IRI relatedEntityIRI = IRI.create(this.mainIRIPrefix + p_relatedEntity);
			if (false == this.mainOntology.containsIndividualInSignature(relatedEntityIRI)) {
				logWarning(String.format("Unknown related entity '%s' for required stream '%s', stream not created.",
						p_entity, p_requirdStream));
				return false;
			}

			OWLIndividual relatedEntityInd = this.dataFactory.getOWLNamedIndividual(relatedEntityIRI);

			assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.ii.aboutRelatedEntity, requiredStream,
					relatedEntityInd);
			addAxiomChange = new AddAxiom(this.mainOntology, assertion);
			changes.add(addAxiomChange);
		}

		// apply changes
		for (OWLOntologyChange change : changes) {
			this.manager.applyChange(change);
		}

		this.dirty = true;

		return true;
	}

	public boolean addRequiredMap(final String p_requirdMap, final String p_namedMapClass, final String p_system,
			final String p_relatedEntity) {
		IRI requiredMapIRI = IRI.create(this.mainIRIPrefix + p_requirdMap);
		if (this.mainOntology.containsIndividualInSignature(requiredMapIRI))
			return false;

		// IRI entityIRI = IRI.create(this.mainIRIPrefix + p_entity);

		// if (p_entity == null || p_entity.isEmpty()
		// || false ==
		// this.mainOntology.containsIndividualInSignature(entityIRI)) {
		// log(String.format("Unknown entity '%s' for required stream '%s', stream not created.",
		// p_entity,
		// p_requirdStream));
		// return false;
		// }

		OWLClass namedMapClass = this.findOWLClass(this.ii.namedMap, p_namedMapClass);

		if (namedMapClass == null) {
			logWarning(String.format("Unknown named map class '%s' for named map '%s', required map not created.",
					p_namedMapClass, p_requirdMap));
			return false;
		}

		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();

		OWLIndividual requiredMap = this.dataFactory.getOWLNamedIndividual(requiredMapIRI);
		OWLIndividual system = this.dataFactory.getOWLNamedIndividual(IRI.create(this.mainIRIPrefix + p_system));

		OWLClassAssertionAxiom ax = this.dataFactory.getOWLClassAssertionAxiom(namedMapClass, requiredMap);
		AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, ax);
		changes.add(addAxiomChange);

		OWLObjectPropertyAssertionAxiom assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(
				this.ii.isSystemOf, system, requiredMap);
		addAxiomChange = new AddAxiom(this.mainOntology, assertion);
		changes.add(addAxiomChange);

		ax = this.dataFactory.getOWLClassAssertionAxiom(this.ii.requiredMap, requiredMap);
		addAxiomChange = new AddAxiom(this.mainOntology, ax);
		changes.add(addAxiomChange);

		// OWLIndividual entityInd =
		// this.dataFactory.getOWLNamedIndividual(entityIRI);
		//
		// assertion =
		// this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.ii.aboutEntity,
		// requiredStream, entityInd);
		// addAxiomChange = new AddAxiom(this.mainOntology, assertion);
		// changes.add(addAxiomChange);

		if (p_relatedEntity != null && false == p_relatedEntity.isEmpty()) {
			IRI relatedEntityIRI = IRI.create(this.mainIRIPrefix + p_relatedEntity);
			if (false == this.mainOntology.containsIndividualInSignature(relatedEntityIRI)) {
				logWarning(String.format(
						"Unknown related entity '%s' for required map '%s', required map not created.",
						p_relatedEntity, p_requirdMap));
				return false;
			}

			OWLIndividual relatedEntityInd = this.dataFactory.getOWLNamedIndividual(relatedEntityIRI);

			assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.ii.aboutRelatedEntity, requiredMap,
					relatedEntityInd);
			addAxiomChange = new AddAxiom(this.mainOntology, assertion);
			changes.add(addAxiomChange);
		}

		// apply changes
		for (OWLOntologyChange change : changes) {
			this.manager.applyChange(change);
		}

		this.dirty = true;

		return true;
	}

	public boolean addSourceNodeClass(final String p_node, final String p_outputs[], final int p_outputsMinSize[],
			final int p_outputsMaxSize[]) {
		return this.addNodeClass(p_node, this.ii.sourceNode, null, null, null, null, null, null, p_outputs,
				p_outputsMinSize, p_outputsMaxSize, null, null, null, null, null, null);
	}

	public boolean addComputationNodeClass(final String p_node, final String p_inputs[], final int p_inputsMinSize[],
			final int p_inputsMaxSize[], final String p_outputs[], final int p_outputsMinSize[],
			final int p_outputsMaxSize[]) {
		return this.addNodeClass(p_node, this.ii.computationNode, p_inputs, p_inputsMinSize, p_inputsMaxSize, null,
				null, null, p_outputs, p_outputsMinSize, p_outputsMaxSize, null, null, null, null, null, null);
	}

	public boolean addIroNodeClass(final String p_node, final String p_inputs[], final int p_inputsMinSize[],
			final int p_inputsMaxSize[], final String p_inputsRelated[], final int p_inputsRelatedMinSize[],
			final int p_inputsRelatedMaxSize[], final String p_outputs[], final int p_outputsMinSize[],
			final int p_outputsMaxSize[]) {
		return this.addNodeClass(p_node, this.ii.iroNode, p_inputs, p_inputsMinSize, p_inputsMaxSize, p_inputsRelated,
				p_inputsRelatedMinSize, p_inputsRelatedMaxSize, p_outputs, p_outputsMinSize, p_outputsMaxSize, null,
				null, null, null, null, null);
	}

	public boolean addMapNodeClass(final String p_node, final String p_inputs[], final int p_inputsMinSize[],
			final int p_inputsMaxSize[], final String p_inputsRelated[], final int p_inputsRelatedMinSize[],
			final int p_inputsRelatedMaxSize[], final String p_inputMaps[], final int p_inputMapsMinSize[],
			final int p_inputMapsMaxSize[], final String p_outputMaps[], final int p_outputMapsMinSize[],
			final int p_outputMapsMaxSize[]) {
		return this.addNodeClass(p_node, this.ii.mapNode, p_inputs, p_inputsMinSize, p_inputsMaxSize, p_inputsRelated,
				p_inputsRelatedMinSize, p_inputsRelatedMaxSize, null, null, null, p_inputMaps, p_inputMapsMinSize,
				p_inputMapsMaxSize, p_outputMaps, p_outputMapsMinSize, p_outputMapsMaxSize);
	}

	public boolean addNodeClass(final String p_node, final OWLClass p_nodeClass, final String p_inputs[],
			final int p_inputsMinSize[], final int p_inputsMaxSize[], final String p_inputsRelated[],
			final int p_inputsRelatedMinSize[], final int p_inputsRelatedMaxSize[], final String p_outputs[],
			final int p_outputsMinSize[], final int p_outputsMaxSize[], final String p_inputMaps[],
			final int p_inputMapsMinSize[], final int p_inputMapsMaxSize[], final String p_outputMaps[],
			final int p_outputMapsMinSize[], final int p_outputMapsMaxSize[]) {
		IRI nodeIRI = IRI.create(this.mainIRIPrefix + p_node);
		if (this.mainOntology.containsClassInSignature(nodeIRI)) {
			logWarning(String.format("Node with name '%s' already exists, will not be created.", p_node));
			return false;
		}

		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();

		// create value scope
		OWLClass node = this.dataFactory.getOWLClass(nodeIRI);
		OWLSubClassOfAxiom axiom = dataFactory.getOWLSubClassOfAxiom(node, p_nodeClass);
		AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, axiom);
		changes.add(addAxiomChange);

		// create inputs
		if (false == this.addPropertiesToNode(node, this.ii.hasInput, this.ii.namedStream, p_inputs, p_inputsMinSize,
				p_inputsMaxSize, changes)) {
			logWarning(String.format("Node '%s' not created.", p_node));
			return false;
		}

		// create related inputs
		if (false == this.addPropertiesToNode(node, this.ii.hasRelatedInput, this.ii.namedStream, p_inputsRelated,
				p_inputsRelatedMinSize, p_inputsRelatedMaxSize, changes)) {
			logWarning(String.format("Node '%s' not created.", p_node));
			return false;
		}

		// create outputs
		if (false == this.addPropertiesToNode(node, this.ii.hasOutput, this.ii.namedStream, p_outputs,
				p_outputsMinSize, p_outputsMaxSize, changes)) {
			logWarning(String.format("Node '%s' not created.", p_node));
			return false;
		}

		// create input maps
		if (false == this.addPropertiesToNode(node, this.ii.hasInputMap, this.ii.namedMap, p_inputMaps,
				p_inputMapsMinSize, p_inputMapsMaxSize, changes)) {
			logWarning(String.format("Node '%s' not created.", p_node));
			return false;
		}

		// create output maps
		if (false == this.addPropertiesToNode(node, this.ii.hasOutputMap, this.ii.namedMap, p_outputMaps,
				p_outputMapsMinSize, p_outputMapsMaxSize, changes)) {
			logWarning(String.format("Node '%s' not created.", p_node));
			return false;
		}

		// apply changes
		this.manager.applyChanges(changes);

		this.dirty = true;

		return true;
	}

	private boolean addPropertiesToNode(OWLClass p_node, OWLObjectProperty p_property, OWLClass p_class,
			final String p_values[], final int p_minSize[], final int p_maxSize[], List<OWLOntologyChange> p_changes) {
		if (p_values != null && p_values.length > 0) {
			// check size
			if (p_values.length != p_minSize.length || p_values.length != p_maxSize.length) {
				logWarning(String.format(
						"Wrong size of inputs '%d' and sizes of min '%s' and max '%s' for object property %s.",
						p_values.length, p_minSize.length, p_maxSize.length, p_property.getIRI()));
				return false;
			}

			for (int i = 0; i < p_values.length; ++i) {
				OWLClass input = this.findOWLClass(p_class, p_values[i]);

				if (input == null) {
					logWarning(String.format("Unknown named input '%s' for class '%s', node not created.", p_values[i],
							p_class.getIRI()));
					return false;
				}

				OWLClassExpression hasInput = null;
				AddAxiom addAxiomChange = null;

				if (p_minSize[i] == p_maxSize[i]) {
					hasInput = this.dataFactory.getOWLObjectExactCardinality(p_minSize[i], p_property, input);
					OWLSubClassOfAxiom ax = this.dataFactory.getOWLSubClassOfAxiom(p_node, hasInput);
					addAxiomChange = new AddAxiom(this.mainOntology, ax);
					p_changes.add(addAxiomChange);
				} else {
					hasInput = this.dataFactory.getOWLObjectMinCardinality(p_minSize[i], p_property, input);
					OWLSubClassOfAxiom ax = this.dataFactory.getOWLSubClassOfAxiom(p_node, hasInput);
					addAxiomChange = new AddAxiom(this.mainOntology, ax);
					p_changes.add(addAxiomChange);

					hasInput = this.dataFactory.getOWLObjectMaxCardinality(p_maxSize[i], p_property, input);
					ax = this.dataFactory.getOWLSubClassOfAxiom(p_node, hasInput);
					addAxiomChange = new AddAxiom(this.mainOntology, ax);
					p_changes.add(addAxiomChange);
				}
			}

			return true;
		}

		return true;
	}

	public boolean addIndividual(final String p_individual, final String p_class) {
		IRI iri = IRI.create(this.mainIRIPrefix + p_individual);
		if (this.mainOntology.containsIndividualInSignature(iri)) {
			this.logWarning(String.format("Individual '%s' for class 's' already exists.", p_individual, p_class));
			return false;
		}

		IRI clsIri = IRI.create(this.mainIRIPrefix + p_class);
		if (false == this.mainOntology.containsClassInSignature(clsIri)) {
			this.logWarning(String.format("Class '%s' for new individual '%s' is unknown.", p_class, p_individual));
			return false;
		}

		OWLClass cls = this.dataFactory.getOWLClass(clsIri);

		OWLIndividual ind = this.dataFactory.getOWLNamedIndividual(iri);
		OWLClassAssertionAxiom ax = this.dataFactory.getOWLClassAssertionAxiom(cls, ind);
		this.manager.addAxiom(this.mainOntology, ax);

		this.dirty = true;

		return true;
	}

	public boolean addIndividual(final String p_individual, final OWLClass p_class) {
		IRI iri = IRI.create(this.mainIRIPrefix + p_individual);
		if (this.mainOntology.containsIndividualInSignature(iri))
			return false;

		OWLIndividual ind = this.dataFactory.getOWLNamedIndividual(iri);
		OWLClassAssertionAxiom ax = this.dataFactory.getOWLClassAssertionAxiom(p_class, ind);
		this.manager.addAxiom(this.mainOntology, ax);

		this.dirty = true;

		return true;
	}

	public boolean addNodeIndividual(final String p_node, final String p_nodeClass, final String p_system,
			final String p_aboutEntity, final String p_aboutRelatedEntity, String[] p_metadatas,
			int[] p_metadataValues, int[] p_metadataValues2, String[] p_metadataGroundings) {
		// check if node exists
		IRI nodeIRI = IRI.create(this.mainIRIPrefix + p_node);
		if (this.mainOntology.containsIndividualInSignature(nodeIRI))
			return false;

		OWLIndividual system = this.findOWLIndividual(this.ii.system, p_system, false);

		if (system == null) {
			system = this.dataFactory.getOWLNamedIndividual(IRI.create(this.mainIRIPrefix + p_system));
			OWLClassAssertionAxiom ax = this.dataFactory.getOWLClassAssertionAxiom(this.ii.system, system);
			// Add this axiom to our ontology - with a convenience method
			this.manager.addAxiom(this.mainOntology, ax);
		}

		// create node
		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();
		OWLIndividual nodeInd = this.dataFactory.getOWLNamedIndividual(nodeIRI);

		OWLClass nodeCls = this.findOWLClass(this.ii.node, p_nodeClass);

		if (nodeCls == null) {
			logWarning(String.format("Unknown node class '%s' for node '%s' in system '%s', node not created.",
					p_nodeClass, p_node, p_system));
			return false;
		}

		OWLClassAssertionAxiom ax = this.dataFactory.getOWLClassAssertionAxiom(nodeCls, nodeInd);
		AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, ax);
		changes.add(addAxiomChange);

		OWLObjectPropertyAssertionAxiom assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(
				this.ii.isSystemOf, system, nodeInd);
		addAxiomChange = new AddAxiom(this.mainOntology, assertion);
		changes.add(addAxiomChange);

		// check about entity
		if (p_aboutEntity != null && false == p_aboutEntity.isEmpty()) {
			OWLIndividual aboutEntity = this.findOWLIndividual(this.ii.entityType, p_aboutEntity, false);

			if (aboutEntity == null) {
				logWarning(String.format("Unknown about entity '%s' for node '%s' in system '%s', node not created.",
						p_aboutEntity, p_node, p_system));
				return false;
			}

			assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.ii.aboutEntity, nodeInd, aboutEntity);
			addAxiomChange = new AddAxiom(this.mainOntology, assertion);
			changes.add(addAxiomChange);
		}

		// check about related entity
		if (p_aboutRelatedEntity != null && false == p_aboutRelatedEntity.isEmpty()) {
			OWLIndividual aboutRelatedEntity = this.findOWLIndividual(this.ii.entityType, p_aboutRelatedEntity, false);

			if (aboutRelatedEntity == null) {
				logWarning(String.format(
						"Unknown about related entity '%s' for node '%s' in system '%s', node not created.",
						p_aboutRelatedEntity, p_node, p_system));
				return false;
			}

			assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.ii.aboutRelatedEntity, nodeInd,
					aboutRelatedEntity);
			addAxiomChange = new AddAxiom(this.mainOntology, assertion);
			changes.add(addAxiomChange);
		}

		// ---
		// Required for structure reasoner
		// assertion =
		// this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.isSystemOfOWLProperty,
		// system.getIndividual(), nodeInd);
		// addAxiomChange = new AddAxiom(this.mainOntology, assertion);
		// changes.add(addAxiomChange);
		// ---

		// add metadata
		for (int i = 0; i < p_metadatas.length; ++i) {
			String metadata = p_metadatas[i];
			String grounding = p_metadataGroundings[i];
			int value = p_metadataValues[i];
			int value2 = p_metadataValues2[i];

			OWLClass metadataCls = this.findOWLClass(this.ii.metadataOWLClass, metadata);

			if (metadataCls == null) {
				logWarning(String.format("Unknown Metadata '%s' for node '%s' in system '%s', node not created.",
						metadata, p_node, p_system));
				return false;
			}

			OWLIndividual metadataInd = this.dataFactory.getOWLNamedIndividual(IRI.create(this.mainIRIPrefix + p_node
					+ metadata));

			ax = this.dataFactory.getOWLClassAssertionAxiom(metadataCls, metadataInd);
			addAxiomChange = new AddAxiom(this.mainOntology, ax);
			changes.add(addAxiomChange);

			assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.ii.hasMetadata, nodeInd, metadataInd);
			addAxiomChange = new AddAxiom(this.mainOntology, assertion);
			changes.add(addAxiomChange);

			// ---
			// Required for structure reasoner
			// assertion =
			// this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.isMetadataOfOWLProperty,
			// metadataInd,
			// nodeInd);
			// addAxiomChange = new AddAxiom(this.mainOntology, assertion);
			// changes.add(addAxiomChange);
			// ---

			// Set Values
			OWLDataPropertyAssertionAxiom dataAssertion = this.dataFactory.getOWLDataPropertyAssertionAxiom(
					this.ii.hasMetadataValue, metadataInd, value);
			addAxiomChange = new AddAxiom(this.mainOntology, dataAssertion);
			changes.add(addAxiomChange);

			dataAssertion = this.dataFactory.getOWLDataPropertyAssertionAxiom(this.ii.hasMetadataValue2, metadataInd,
					value2);
			addAxiomChange = new AddAxiom(this.mainOntology, dataAssertion);
			changes.add(addAxiomChange);

			// set grounding
			OWLIndividual metadataGrounding = this.findOWLIndividual(this.ii.aspMetadataGroundingOWLClass, grounding,
					false);

			if (metadataGrounding == null) {
				logWarning(String
						.format("Unknown Metadata grounding '%s' of grounding '%s' for node '%s' in system '%s', node not created.",
								grounding, metadata, p_node, p_system));
				return false;
			}

			assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.ii.hasGrounding, metadataInd,
					metadataGrounding);
			addAxiomChange = new AddAxiom(this.mainOntology, assertion);
			changes.add(addAxiomChange);

			// ---
			// Required for structure reasoner
			// assertion =
			// this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.isGroundingOfOWLProperty,
			// metadataGrounding, metadataInd);
			// addAxiomChange = new AddAxiom(this.mainOntology, assertion);
			// changes.add(addAxiomChange);
			// ---
		}

		// apply changes
		this.manager.applyChanges(changes);

		this.dirty = true;

		return true;
	}

	public boolean addIROIndividual(final String p_iro, final String p_iroClass, final String p_system,
			String[] p_metadatas, int[] p_metadataValues, String[] p_metadataGroundings) {
		// check if iro exists
		IRI iroIRI = IRI.create(this.mainIRIPrefix + p_iro);
		if (this.mainOntology.containsIndividualInSignature(iroIRI))
			return false;

		OWLIndividual system = this.findOWLIndividual(this.ii.system, p_system, false);

		if (system == null) {
			system = this.dataFactory.getOWLNamedIndividual(IRI.create(this.mainIRIPrefix + p_system));
			OWLClassAssertionAxiom ax = this.dataFactory.getOWLClassAssertionAxiom(this.ii.system, system);
			// Add this axiom to our ontology - with a convenience method
			this.manager.addAxiom(this.mainOntology, ax);
		}

		// create iro
		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();
		OWLIndividual iroInd = this.dataFactory.getOWLNamedIndividual(iroIRI);

		OWLClass iroCls = this.findOWLClass(this.ii.iroNode, p_iroClass);

		if (iroCls == null) {
			logWarning(String.format("Unknown IRO class '%s' for IRO '%s' in system '%s', IRO not created.",
					p_iroClass, p_iro, p_system));
			return false;
		}

		OWLClassAssertionAxiom ax = this.dataFactory.getOWLClassAssertionAxiom(iroCls, iroInd);
		AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, ax);
		changes.add(addAxiomChange);

		OWLObjectPropertyAssertionAxiom assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(
				this.ii.hasSystem, iroInd, system);
		addAxiomChange = new AddAxiom(this.mainOntology, assertion);
		changes.add(addAxiomChange);

		// ---
		// Required for structure reasoner
		// assertion =
		// this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.isSystemOfOWLProperty,
		// system.getIndividual(), nodeInd);
		// addAxiomChange = new AddAxiom(this.mainOntology, assertion);
		// changes.add(addAxiomChange);
		// ---

		// add metadata
		for (int i = 0; i < p_metadatas.length; ++i) {
			String metadata = p_metadatas[i];
			String grounding = p_metadataGroundings[i];
			int value = p_metadataValues[i];

			OWLClass metadataCls = this.findOWLClass(this.ii.metadataOWLClass, metadata);

			if (metadataCls == null) {
				logWarning(String.format("Unknown Metadata '%s' for IRO '%s' in system '%s', IRO not created.",
						metadata, p_iro, p_system));
				return false;
			}

			OWLIndividual metadataInd = this.dataFactory.getOWLNamedIndividual(IRI.create(this.mainIRIPrefix + p_iro
					+ metadata));

			ax = this.dataFactory.getOWLClassAssertionAxiom(metadataCls, metadataInd);
			addAxiomChange = new AddAxiom(this.mainOntology, ax);
			changes.add(addAxiomChange);

			assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.ii.hasMetadata, iroInd, metadataInd);
			addAxiomChange = new AddAxiom(this.mainOntology, assertion);
			changes.add(addAxiomChange);

			// ---
			// Required for structure reasoner
			// assertion =
			// this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.isMetadataOfOWLProperty,
			// metadataInd,
			// nodeInd);
			// addAxiomChange = new AddAxiom(this.mainOntology, assertion);
			// changes.add(addAxiomChange);
			// ---

			OWLDataPropertyAssertionAxiom dataAssertion = this.dataFactory.getOWLDataPropertyAssertionAxiom(
					this.ii.hasMetadataValue, metadataInd, value);
			addAxiomChange = new AddAxiom(this.mainOntology, dataAssertion);
			changes.add(addAxiomChange);

			// set grounding
			OWLIndividual metadataGrounding = this.findOWLIndividual(this.ii.aspMetadataGroundingOWLClass, grounding,
					false);

			if (metadataGrounding == null) {
				logWarning(String
						.format("Unknown Metadata grounding '%s' of grounding '%s' for IRO '%s' in system '%s', IRO not created.",
								grounding, metadata, p_iro, p_system));
				return false;
			}

			assertion = this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.ii.hasGrounding, metadataInd,
					metadataGrounding);
			addAxiomChange = new AddAxiom(this.mainOntology, assertion);
			changes.add(addAxiomChange);

			// ---
			// Required for structure reasoner
			// assertion =
			// this.dataFactory.getOWLObjectPropertyAssertionAxiom(this.isGroundingOfOWLProperty,
			// metadataGrounding, metadataInd);
			// addAxiomChange = new AddAxiom(this.mainOntology, assertion);
			// changes.add(addAxiomChange);
			// ---
		}

		// apply changes
		for (OWLOntologyChange change : changes) {
			this.manager.applyChange(change);
		}

		this.dirty = true;

		return true;
	}

	public boolean addOntologyIRI(final String p_iri) {
		if (false == this.iriMapping.contains(p_iri))
			this.iriMapping.add(p_iri);

		return this.ontologyIries.add(p_iri);
	}

	public boolean removeOntologyIRI(final String p_iri) {
		return this.ontologyIries.remove(p_iri);
	}

	public String readInformationStructureAsASP() {
		// long start = System.currentTimeMillis();
		Set<OWLOntology> onts = new HashSet<OWLOntology>();
		onts.add(this.mainOntology);

		onts.addAll(this.imports);

		InfoStructureVisitor isv = new InfoStructureVisitor(this, onts, this.getReasoner(), this.ii);

		Set<OWLClass> classes = this.getReasoner().getSubClasses(this.ii.entityType, false).getFlattened();

		for (OWLClassExpression cls : classes) {
			cls.accept(isv);
		}
		// long stop = System.currentTimeMillis();
		// if (stop - start > 100) {
		// System.out.println("Time readInformationStructureAsASP " + (mid -
		// start));
		// System.out.println("Time readInformationStructureAsASP " + (mid2 -
		// start));
		// System.out.println("Time readInformationStructureAsASP " + (stop -
		// start));
		// }

		return isv.toString();
	}

	public final String readRepresentationsAsCSV() {
		StringBuilder sb = new StringBuilder();

		Set<OWLOntology> onts = new HashSet<OWLOntology>();
		onts.add(this.mainOntology);
		onts.addAll(this.imports);

		RepresentationVisitor rv = new RepresentationVisitor(this, onts, this.getReasoner(), this.ii);
		Set<OWLClass> classes = this.getReasoner().getSubClasses(this.ii.representation, false).getFlattened();

		for (OWLClassExpression cls : classes) {
			rv.reset();
			cls.accept(rv);
			if (rv.addToStringBuilder(sb) > 0)
				sb.append("|\n");
		}

		return sb.toString();
	}

	public String[][] readNodesAndIROsAsASP(String p_system) {
		Set<OWLOntology> onts = new HashSet<OWLOntology>();
		onts.add(this.mainOntology);
		onts.addAll(this.imports);

		OWLNamedIndividual system = this.findOWLIndividual(this.ii.system, p_system, true);

		if (system == null) {
			system = this.dataFactory.getOWLNamedIndividual(IRI.create(this.mainIRIPrefix + p_system));
			OWLClassAssertionAxiom ax = this.dataFactory.getOWLClassAssertionAxiom(this.ii.system, system);
			// Add this axiom to our ontology - with a convenience method
			this.manager.addAxiom(this.mainOntology, ax);
		}

		NodeIROVisitor niv = new NodeIROVisitor(this, onts, this.getReasoner(), this.ii);

		List<List<String>> result = niv.readInformation(system);

		String[][] resultArray = new String[result.size()][0];

		for (int i = 0; i < result.size(); ++i) {
			List<String> lst = result.get(i);
			String[] arr = new String[lst.size()];
			for (int j = 0; j < lst.size(); ++j) {
				arr[j] = lst.get(j);
			}
			resultArray[i] = arr;
		}

		return resultArray;
	}

	public String[] getSystems() {
		Set<OWLNamedIndividual> systems = this.getReasoner().getInstances(this.ii.system, true).getFlattened();

		String[] result = new String[systems.size()];
		int i = 0;

		for (OWLNamedIndividual ind : systems) {
			result[i] = ind.getIRI().toString();
			++i;
		}

		return result;
	}

	public boolean initReasoner(boolean p_force) {
		if (p_force || this.dirty || this.internalReasoner == null) {
			this.internalReasoner = this.reasonerFactory.createReasoner(this.mainOntology);
			return true;
		}

		return false;
	}

	private OWLReasoner getReasoner() {
		if (this.internalReasoner == null) {
			this.internalReasoner = this.reasonerFactory.createReasoner(this.mainOntology);
			this.dirty = false;
		} else if (this.dirty) {
			// this.internalReasoner.dispose();
			// this.internalReasoner =
			// this.reasonerFactory.createReasoner(this.mainOntology);
			((StructuralReasoner) this.internalReasoner).prepareReasoner();
			this.dirty = false;
		}

		return this.internalReasoner;
	}

	private OWLNamedIndividual findOWLIndividual(final OWLClass p_class, final String p_name, boolean p_fullName) {
		Set<OWLClass> subs = this.getAllLeafs(p_class);

		for (OWLClass sub : subs) {
			Set<OWLNamedIndividual> inds = this.getReasoner().getInstances(sub, true).getFlattened();

			for (OWLNamedIndividual ind : inds) {
				if (p_fullName) {
					if (ind.getIRI().toString().equals(p_name))
						return ind;
				} else {
					if (ind.getIRI().toString().endsWith("#" + p_name))
						return ind;
				}
			}
		}

		return null;
	}

	private OWLClass findOWLClass(final OWLClass p_class, final String p_name) {
		Set<OWLClass> subs = this.getAllLeafs(p_class);

		for (OWLClass cls : subs) {
			if (cls.getIRI().toString().endsWith("#" + p_name))
				return cls;
		}

		return null;
	}

	protected Set<OWLClass> getAllLeafs(final OWLClass p_start) {
		Set<OWLClass> subClasses = this.getReasoner().getSubClasses(p_start, true).getFlattened();
		Set<OWLClass> results = new HashSet<OWLClass>();

		for (OWLClass c : subClasses) {
			if (c.isOWLNothing()) {
				results.add(p_start);
			}

			results.addAll(this.getAllLeafs(c));
		}

		return results;
	}

	protected boolean isSubClassOf(final OWLClassExpression p_child, final OWLClass p_parent) {
		if (p_child == null)
			return false;

		// owlapi 4.0 remove this
		if (p_child.isAnonymous()) {
			return false;
		}

		if (p_child.equals(p_parent))
			return true;

		Set<OWLClass> superClasses = this.getReasoner().getSuperClasses(p_child, true).getFlattened();

		for (OWLClass c : superClasses) {
			if (c.equals(p_parent))
				return true;

			if (this.isSubClassOf(c, p_parent))
				return true;
		}

		return false;
	}

	public int getSomeMinCardinality() {
		return someMinCardinality;
	}

	public void setSomeMinCardinality(int someMinCardinality) {
		this.someMinCardinality = someMinCardinality;
	}

	public int getSomeMaxCardinality() {
		return someMaxCardinality;
	}

	public void setSomeMaxCardinality(int someMaxCardinality) {
		this.someMaxCardinality = someMaxCardinality;
	}

	public void log(LogLevel ll, String msg) {
		if (logLevel.getValue() < ll.getValue())
			return;

		final String date = DATE_FORMAT.format(new Date());
		final String className = this.getClass().getSimpleName();
		String llstr = ll.toString();

		if (llstr.length() < 5)
			llstr = llstr + " ";

		System.out.printf("%s %s [%s] %s\n", date, llstr, className, msg);
	}

	private void logError(String msg) {
		log(LogLevel.Error, msg);
	}

	private void logWarning(String msg) {
		log(LogLevel.Warn, msg);
	}

	private void logDebug(String msg) {
		log(LogLevel.Debug, msg);
	}

	private void logInfo(String msg) {
		log(LogLevel.Info, msg);
	}

	public int getDefaultMinCardinality() {
		return defaultMinCardinality;
	}

	public void setDefaultMinCardinality(int defaultMinCardinality) {
		this.defaultMinCardinality = defaultMinCardinality;
	}

	public int getDefaultMaxCardinality() {
		return defaultMaxCardinality;
	}

	public void setDefaultMaxCardinality(int defaultMaxCardinality) {
		this.defaultMaxCardinality = defaultMaxCardinality;
	}

	public void setLogLevel(int ll) {
		logLevel = LogLevel.values()[ll];
	}

	public int getLogLevel() {
		return logLevel.getValue();
	}

	public String iRIShortName(IRI p_iri) {
		String[] values = p_iri.toString().split("#");
		int index = iriMapping.indexOf(values[0]);

		if (index < 0) {
			// index = IRI_MAPPING.size();
			// IRI_MAPPING.add(values[0]);

			this.logWarning(String.format("Unkonwn IRI '%s' from '%s', no mapping to short iri", values[0], p_iri));
		}

		// System.out.println(index + "   " + values[1] + "   " + values[0] +
		// "   " + (IRI_MAPPING.size()));

		// return "o" + index + "_" + values[1];
		return p_iri.toString();
	}

	public String[] getOntologyIriMapping() {
		String[] stockArr = new String[iriMapping.size()];
		stockArr = iriMapping.toArray(stockArr);
		return stockArr;
	}

	private class MemoryMonitor implements Runnable {
		IceOntologyInterface ioi;

		public MemoryMonitor(IceOntologyInterface ioi) {
			super();
			this.ioi = ioi;
		}

		@Override
		public void run() {
			while (ioi.memoryMonitorState) {
				long t = Runtime.getRuntime().totalMemory();
				long m = Runtime.getRuntime().maxMemory();
				long f = Runtime.getRuntime().freeMemory();

				// if (t > ioi.memoryUsage[0])
				// ioi.memoryUsage[0] = t;
				if (t - f > ioi.memoryUsage[0])
					ioi.memoryUsage[0] = t - f;
				// if (f > ioi.memoryUsage[2])
				// ioi.memoryUsage[2] = f;

				try {
					Thread.sleep(5);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}
}
