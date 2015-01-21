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
import org.semanticweb.owlapi.model.OWLClassExpression;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLDataPropertyAssertionAxiom;
import org.semanticweb.owlapi.model.OWLIndividual;
import org.semanticweb.owlapi.model.OWLNamedIndividual;
import org.semanticweb.owlapi.model.OWLObjectPropertyAssertionAxiom;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyChange;
import org.semanticweb.owlapi.model.OWLOntologyCreationException;
import org.semanticweb.owlapi.model.OWLOntologyIRIMapper;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.model.OWLSubClassOfAxiom;
import org.semanticweb.owlapi.reasoner.OWLReasoner;
import org.semanticweb.owlapi.reasoner.OWLReasonerFactory;
import org.semanticweb.owlapi.reasoner.structural.StructuralReasonerFactory;
import org.semanticweb.owlapi.util.AutoIRIMapper;

public class IceOntologyInterface {
	private OWLOntologyManager manager;
	private OWLOntology mainOntology;
	private Set<OWLOntology> imports;
	private OWLReasonerFactory reasonerFactory;
	private OWLReasoner internalReasoner;
	private OWLDataFactory dataFactory;

	private IceIris ii;

	// private OWLClass systemOWLClass;
	// private OWLClass nodeOWLClass;
	// private OWLClass iroOWLClass;
	// private OWLClass metadataOWLClass;
	// private OWLClass entityTypeOWLClass;
	// private OWLClass aspMetadataGroundingOWLClass;
	// private OWLObjectProperty hasSystemOWLProperty;
	// private OWLObjectProperty isSystemOfOWLProperty;
	// private OWLObjectProperty hasMetadataOWLProperty;
	// private OWLObjectProperty isMetadataOfOWLProperty;
	// private OWLObjectProperty hasGroundingOWLProperty;
	// private OWLObjectProperty isGroundingOfOWLProperty;
	// private OWLDataProperty hasMetadataValueOWLDataProperty;

	private List<String> ontologyIries;
	private boolean dirty;

	private int someMinCardinality;
	private int someMaxCardinality;
	private int defaultMinCardinality;
	private int defaultMaxCardinality;

	private boolean logging;

	private String mainIRI;
	private String mainIRIPrefix;

	public IceOntologyInterface() {
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
		this.logging = true;
		// System.gc();
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

			// Fetching ontology things: class, properties, usw.
			this.ii = new IceIris(this.dataFactory);

			this.dirty = true;

			return true;
		} catch (Exception e) {
			this.log("Exception during loading the ontology " + e.getMessage());
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

		// Fetching ontology things: class, properties, usw.
		this.ii = new IceIris(this.dataFactory);

		this.dirty = true;

		return true;
	}

	public boolean saveOntology(final String p_saveTo) {
		try {
			File file = new File(p_saveTo);
			manager.saveOntology(this.mainOntology, IRI.create(file.toURI()));
			return true;
		} catch (Exception e) {
			this.log("Exception during saving the ontology " + e.getMessage());
			return false;
		}
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
			log(String.format("No system '%s' found, isSystemOf will not be established", p_system));
			return false;
		}

		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();

		for (String toAdd : p_toAdd) {
			OWLIndividual nodeInd = this.findOWLIndividual(this.ii.node, toAdd, false);

			if (nodeInd == null) {
				log(String.format("No node '%s' found, isSystemOf for system '%s' will not be established.", toAdd,
						p_system));
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
				log(String.format("Unknown entity scope class '%s' for entity '%s', entity type not created.",
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
				log(String.format("Unknown entity scope class '%s' for entity '%s', entity type not created.",
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
				log(String.format("Unknown representation class '%s' for entity scope '%s', entity scope not created.",
						representation, p_entityScope));
				return false;
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

	public boolean addValueScope(final String p_superValueScope, final String p_valueScope) {
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

		// create value scope
		OWLClass valueScope = this.dataFactory.getOWLClass(valueScopeIRI);
		OWLSubClassOfAxiom axiom = dataFactory.getOWLSubClassOfAxiom(valueScope, superValueScope);
		AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, axiom);
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
				log(String.format("Unknown dimension class '%s' for representation '%s', representation not created.",
						dimension, p_representation));
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

	public boolean addNamedStream(final String p_stream, final String p_entityScope, final String p_representation) {
		IRI streamIRI = IRI.create(this.mainIRIPrefix + p_stream);
		if (this.mainOntology.containsClassInSignature(streamIRI))
			return false;

		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();

		// featch scope and representation
		OWLClass entityScope = this.findOWLClass(this.ii.entityScope, p_entityScope);
		OWLClass representation = this.findOWLClass(this.ii.representation, p_representation);

		if (entityScope == null) {
			log(String.format("Unknown entity scope class '%s' for stream '%s', stream not created.", p_entityScope,
					p_stream));
			return false;
		}
		if (representation == null) {
			log(String.format("Unknown representation class '%s' for stream '%s', stream not created.",
					p_representation, p_stream));
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

	public boolean addRequiredStream(final String p_requirdStream, final String p_namedStreamClass,
			final String p_system, final String p_entity, final String p_relatedEntity) {
		IRI requiredStreamIRI = IRI.create(this.mainIRIPrefix + p_requirdStream);
		if (this.mainOntology.containsIndividualInSignature(requiredStreamIRI))
			return false;

		IRI entityIRI = IRI.create(this.mainIRIPrefix + p_entity);

		if (p_entity == null || p_entity.isEmpty()
				|| false == this.mainOntology.containsIndividualInSignature(entityIRI)) {
			log(String.format("Unknown entity '%s' for required stream '%s', stream not created.", p_entity,
					p_requirdStream));
			return false;
		}

		OWLClass namedStreamClass = this.findOWLClass(this.ii.namedStream, p_namedStreamClass);

		if (namedStreamClass == null) {
			log(String.format("Unknown named stream class '%s' for named stream '%s', stream not created.",
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
				log(String.format("Unknown related entity '%s' for required stream '%s', stream not created.",
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

	public boolean addSourceNodeClass(final String p_node, final String p_outputs[], final int p_outputsMinSize[],
			final int p_outputsMaxSize[]) {
		return this.addNodeClass(p_node, this.ii.sourceNode, null, null, null, null, null, null, p_outputs,
				p_outputsMinSize, p_outputsMaxSize);
	}

	public boolean addComputationNodeClass(final String p_node, final String p_inputs[], final int p_inputsMinSize[],
			final int p_inputsMaxSize[], final String p_outputs[], final int p_outputsMinSize[],
			final int p_outputsMaxSize[]) {
		return this.addNodeClass(p_node, this.ii.computationNode, p_inputs, p_inputsMinSize, p_inputsMaxSize, null,
				null, null, p_outputs, p_outputsMinSize, p_outputsMaxSize);
	}

	public boolean addIroNodeClass(final String p_node, final String p_inputs[], final int p_inputsMinSize[],
			final int p_inputsMaxSize[], final String p_inputsRelated[], final int p_inputsRelatedMinSize[],
			final int p_inputsRelatedMaxSize[], final String p_outputs[], final int p_outputsMinSize[],
			final int p_outputsMaxSize[]) {
		return this.addNodeClass(p_node, this.ii.iroNode, p_inputs, p_inputsMinSize, p_inputsMaxSize, p_inputsRelated,
				p_inputsRelatedMinSize, p_inputsRelatedMaxSize, p_outputs, p_outputsMinSize, p_outputsMaxSize);
	}

	public boolean addNodeClass(final String p_node, final OWLClass p_nodeClass, final String p_inputs[],
			final int p_inputsMinSize[], final int p_inputsMaxSize[], final String p_inputsRelated[],
			final int p_inputsRelatedMinSize[], final int p_inputsRelatedMaxSize[], final String p_outputs[],
			final int p_outputsMinSize[], final int p_outputsMaxSize[]) {
		IRI nodeIRI = IRI.create(this.mainIRIPrefix + p_node);
		if (this.mainOntology.containsClassInSignature(nodeIRI))
			return false;

		if (p_inputs != null
				&& (p_inputs.length != p_inputsMinSize.length || p_inputs.length != p_inputsMaxSize.length)) {
			log(String.format("Wrong size of inputs '%d' and sizes of min '%s' and max '%s', node not created.",
					p_node, p_inputs.length, p_inputsMinSize.length, p_inputsMaxSize.length));
			return false;
		}

		if (p_inputsRelated != null
				&& (p_inputsRelated.length != p_inputsRelatedMinSize.length || p_inputs.length != p_inputsRelatedMaxSize.length)) {
			log(String.format(
					"Wrong size of related inputs '%d' and sizes of min '%s' and max '%s', node not created.", p_node,
					p_inputsRelated.length, p_inputsRelatedMinSize.length, p_inputsRelatedMaxSize.length));
			return false;
		}

		if (p_outputs != null
				&& (p_outputs.length != p_outputsMinSize.length || p_outputs.length != p_outputsMaxSize.length)) {
			log(String.format("Wrong size of outputs '%d' and sizes of min '%s' and max '%s', node not created.",
					p_node, p_outputs.length, p_outputsMinSize.length, p_outputsMaxSize.length));
			return false;
		}

		List<OWLOntologyChange> changes = new ArrayList<OWLOntologyChange>();

		// create value scope
		OWLClass node = this.dataFactory.getOWLClass(nodeIRI);
		OWLSubClassOfAxiom axiom = dataFactory.getOWLSubClassOfAxiom(node, p_nodeClass);
		AddAxiom addAxiomChange = new AddAxiom(this.mainOntology, axiom);
		changes.add(addAxiomChange);

		// create inputs
		if (p_inputs != null) {
			for (int i = 0; i < p_inputs.length; ++i) {
				OWLClass input = this.findOWLClass(this.ii.namedStream, p_inputs[i]);

				if (input == null) {
					log(String.format("Unknown named input stream class '%s' for node '%s', node not created.",
							p_inputs[i], p_node));
					return false;
				}

				OWLClassExpression hasInput = null;

				if (p_inputsMinSize[i] == p_inputsMaxSize[i]) {
					hasInput = this.dataFactory.getOWLObjectExactCardinality(p_inputsMinSize[i], this.ii.hasInput,
							input);
					OWLSubClassOfAxiom ax = this.dataFactory.getOWLSubClassOfAxiom(node, hasInput);
					addAxiomChange = new AddAxiom(this.mainOntology, ax);
					changes.add(addAxiomChange);
				} else {
					hasInput = this.dataFactory.getOWLObjectMinCardinality(p_inputsMinSize[i], this.ii.hasInput, input);
					OWLSubClassOfAxiom ax = this.dataFactory.getOWLSubClassOfAxiom(node, hasInput);
					addAxiomChange = new AddAxiom(this.mainOntology, ax);
					changes.add(addAxiomChange);

					hasInput = this.dataFactory.getOWLObjectMaxCardinality(p_inputsMaxSize[i], this.ii.hasInput, input);
					ax = this.dataFactory.getOWLSubClassOfAxiom(node, hasInput);
					addAxiomChange = new AddAxiom(this.mainOntology, ax);
					changes.add(addAxiomChange);
				}
			}
		}

		// create related inputs
		if (p_inputsRelated != null) {
			for (int i = 0; i < p_inputsRelated.length; ++i) {
				OWLClass inputRelated = this.findOWLClass(this.ii.namedStream, p_inputsRelated[i]);

				if (inputRelated == null) {
					log(String.format("Unknown named related input stream class '%s' for node '%s', node not created.",
							p_inputsRelated[i], p_node));
					return false;
				}

				OWLClassExpression hasInputRelated = null;

				if (p_inputsRelatedMinSize[i] == p_inputsRelatedMaxSize[i]) {
					hasInputRelated = this.dataFactory.getOWLObjectExactCardinality(p_inputsRelatedMinSize[i],
							this.ii.hasRelatedInput, inputRelated);
					OWLSubClassOfAxiom ax = this.dataFactory.getOWLSubClassOfAxiom(node, hasInputRelated);
					addAxiomChange = new AddAxiom(this.mainOntology, ax);
					changes.add(addAxiomChange);
				} else {
					hasInputRelated = this.dataFactory.getOWLObjectMinCardinality(p_inputsRelatedMinSize[i],
							this.ii.hasRelatedInput, inputRelated);
					OWLSubClassOfAxiom ax = this.dataFactory.getOWLSubClassOfAxiom(node, hasInputRelated);
					addAxiomChange = new AddAxiom(this.mainOntology, ax);
					changes.add(addAxiomChange);

					hasInputRelated = this.dataFactory.getOWLObjectMaxCardinality(p_inputsRelatedMaxSize[i],
							this.ii.hasRelatedInput, inputRelated);
					ax = this.dataFactory.getOWLSubClassOfAxiom(node, hasInputRelated);
					addAxiomChange = new AddAxiom(this.mainOntology, ax);
					changes.add(addAxiomChange);
				}
			}
		}

		// create outputs
		if (p_outputs != null) {
			for (int i = 0; i < p_outputs.length; ++i) {
				OWLClass output = this.findOWLClass(this.ii.namedStream, p_outputs[i]);

				if (output == null) {
					log(String.format("Unknown named output stream class '%s' for node '%s', node not created.",
							p_outputs[i], p_node));
					return false;
				}

				OWLClassExpression hasOutput = null;

				if (p_outputsMinSize[i] == p_outputsMaxSize[i]) {
					hasOutput = this.dataFactory.getOWLObjectExactCardinality(p_outputsMinSize[i], this.ii.hasOutput,
							output);
					OWLSubClassOfAxiom ax = this.dataFactory.getOWLSubClassOfAxiom(node, hasOutput);
					addAxiomChange = new AddAxiom(this.mainOntology, ax);
					changes.add(addAxiomChange);
				} else {
					hasOutput = this.dataFactory.getOWLObjectMinCardinality(p_outputsMinSize[i], this.ii.hasOutput,
							output);
					OWLSubClassOfAxiom ax = this.dataFactory.getOWLSubClassOfAxiom(node, hasOutput);
					addAxiomChange = new AddAxiom(this.mainOntology, ax);
					changes.add(addAxiomChange);

					hasOutput = this.dataFactory.getOWLObjectMaxCardinality(p_outputsMaxSize[i], this.ii.hasOutput,
							output);
					ax = this.dataFactory.getOWLSubClassOfAxiom(node, hasOutput);
					addAxiomChange = new AddAxiom(this.mainOntology, ax);
					changes.add(addAxiomChange);
				}
			}
		}

		// apply changes
		this.manager.applyChanges(changes);

		this.dirty = true;

		return true;
	}

	public boolean addIndividual(final String p_individual, final String p_class) {
		IRI iri = IRI.create(this.mainIRIPrefix + p_individual);
		if (this.mainOntology.containsIndividualInSignature(iri)) {
			this.log(String.format("Individual '%s' for class 's' already exists.", p_individual, p_class));
			return false;
		}

		IRI clsIri = IRI.create(this.mainIRIPrefix + p_class);
		if (false == this.mainOntology.containsClassInSignature(clsIri)) {
			this.log(String.format("Class '%s' for new individual '%s' is unknown.", p_class, p_individual));
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
			log(String.format("Unknown node class '%s' for node '%s' in system '%s', node not created.", p_nodeClass,
					p_node, p_system));
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
				log(String.format("Unknown about entity '%s' for node '%s' in system '%s', node not created.",
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
				log(String.format("Unknown about related entity '%s' for node '%s' in system '%s', node not created.",
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
				log(String.format("Unknown Metadata '%s' for node '%s' in system '%s', node not created.", metadata,
						p_node, p_system));
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
				System.out
						.println(String
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
			log(String.format("Unknown IRO class '%s' for IRO '%s' in system '%s', IRO not created.", p_iroClass,
					p_iro, p_system));
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
				log(String.format("Unknown Metadata '%s' for IRO '%s' in system '%s', IRO not created.", metadata,
						p_iro, p_system));
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
				System.out
						.println(String
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
		long mid = System.currentTimeMillis();

		Set<OWLClass> classes = this.getReasoner().getSubClasses(this.ii.entityType, false).getFlattened();
		long mid2 = System.currentTimeMillis();

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
		if (this.dirty || this.internalReasoner == null) {
			this.internalReasoner = this.reasonerFactory.createReasoner(this.mainOntology);
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

	private void log(String p_msg) {
		if (false == this.logging)
			return;

		System.out.println("java     " + p_msg);
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

	public boolean isLogging() {
		return logging;
	}

	public void setLogging(boolean logging) {
		this.logging = logging;
	}
}
