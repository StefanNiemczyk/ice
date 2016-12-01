package de.unikassel.vs.ice;

import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLDataProperty;
import org.semanticweb.owlapi.model.OWLObjectProperty;

public class IceIris {
	public static final String ICE_IRI = "http://vs.uni-kassel.de/Ice";
	public static final String ICE_IRI_PREFIX = ICE_IRI + "#";

	public final OWLClass node;
	public final OWLClass computationNode;
	public final OWLClass sourceNode;
	public final OWLClass transformationNode;
	public final OWLClass setNode;
	public final OWLClass requiredStream;
	public final OWLClass requiredSet;
	public final OWLClass system;
	public final OWLClass entityType;
	public final OWLClass scope;
	public final OWLClass entityScope;
	public final OWLClass representation;
	public final OWLClass compositeRepresentation;
	public final OWLClass relatedRepresentation;
	public final OWLClass valueScope;
	public final OWLClass stream;
	public final OWLClass namedStream;
	public final OWLClass namedSet;
	public final OWLClass groundingOWLClass;
	public final OWLClass metadataOWLClass;
	public final OWLClass entityTypeOWLClass;
	public final OWLClass aspMetadataGroundingOWLClass;

	public final OWLObjectProperty hasSystem;
	public final OWLObjectProperty hasInput;
	public final OWLObjectProperty hasInputSet;
	public final OWLObjectProperty hasRelatedInput;
	public final OWLObjectProperty hasOutput;
	public final OWLObjectProperty hasOutputSet;
	public final OWLObjectProperty isSystemOf;
	public final OWLObjectProperty isStreamOf;
	public final OWLObjectProperty isSetOf;
	public final OWLObjectProperty isGroundingOf;
	public final OWLObjectProperty onlyEntity;
	public final OWLObjectProperty hasRepresentation;
	public final OWLObjectProperty hasScope;
	public final OWLObjectProperty hasDimension;
	public final OWLObjectProperty hasRelatedDimension;
	public final OWLObjectProperty hasUnit;
	public final OWLObjectProperty hasStreamMetadata;
	public final OWLObjectProperty hasMetadata;
	public final OWLObjectProperty improveInformationMetadata;
	public final OWLObjectProperty impairInformationMetadata;
	public final OWLObjectProperty hasGrounding;
	public final OWLObjectProperty aboutEntity;
	public final OWLObjectProperty aboutRelatedEntity;

	public final OWLDataProperty hasGroundingValue;
	public final OWLDataProperty hasMetadataValue;
	public final OWLDataProperty hasMetadataValue2;
	public final OWLDataProperty hasConfiguration;

	public IceIris(final OWLDataFactory p_dataFactory) {
		this.entityType = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "EntityType"));
		this.scope = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "Scope"));
		this.entityScope = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "EntityScope"));
		this.representation = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "Representation"));
		this.compositeRepresentation = p_dataFactory
				.getOWLClass(IRI.create(ICE_IRI_PREFIX + "CompositeRepresentation"));
		this.relatedRepresentation = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "RelatedRepresentation"));
		this.valueScope = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "ValueScope"));
		this.node = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "Node"));
		this.computationNode = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "ComputationNode"));
		this.sourceNode = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "SourceNode"));
		this.transformationNode = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "TransformationNode"));
		this.setNode = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "SetNode"));
		this.requiredStream = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "RequiredStream"));
		this.requiredSet = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "RequiredSet"));
		this.system = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "System"));
		this.stream = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "Stream"));
		this.namedStream = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "NamedStream"));
		this.namedSet = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "NamedSet"));
		this.groundingOWLClass = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "Grounding"));
		this.metadataOWLClass = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "Metadata"));
		this.entityTypeOWLClass = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "EntityType"));
		this.aspMetadataGroundingOWLClass = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX
				+ "ASPMetadataGrounding"));

		this.hasSystem = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasSystem"));
		this.hasInput = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasInput"));
		this.hasInputSet = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasInputSet"));
		this.hasRelatedInput = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasRelatedInput"));
		this.hasOutput = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasOutput"));
		this.hasOutputSet = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasOutputSet"));
		this.isSystemOf = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "isSystemOf"));
		this.isStreamOf = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "isStreamOf"));
		this.isSetOf = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "isSetOf"));
		this.isGroundingOf = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "isGroundingOf"));
		this.onlyEntity = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "onlyEntity"));
		this.hasRepresentation = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasRepresentation"));
		this.hasScope = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasScope"));
		this.hasDimension = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasDimension"));
		this.hasRelatedDimension = p_dataFactory.getOWLObjectProperty(IRI
				.create(ICE_IRI_PREFIX + "hasRelatedDimension"));
		this.hasUnit = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasUnit"));
		this.hasStreamMetadata = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasStreamMetadata"));
		this.hasMetadata = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasMetadata"));
		this.improveInformationMetadata = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX
				+ "improveInformationMetadata"));
		this.impairInformationMetadata = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX
				+ "impairInformationMetadata"));
		this.hasGrounding = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasGrounding"));
		this.aboutEntity = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "aboutEntity"));
		this.aboutRelatedEntity = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "aboutRelatedEntity"));

		this.hasGroundingValue = p_dataFactory.getOWLDataProperty(IRI.create(ICE_IRI_PREFIX + "hasGroundingValue"));
		this.hasMetadataValue = p_dataFactory.getOWLDataProperty(IRI.create(ICE_IRI_PREFIX + "hasMetadataValue"));
		this.hasMetadataValue2 = p_dataFactory.getOWLDataProperty(IRI.create(ICE_IRI_PREFIX + "hasMetadataValue2"));
		this.hasConfiguration = p_dataFactory.getOWLDataProperty(IRI.create(ICE_IRI_PREFIX + "hasConfiguration"));
	}
}
