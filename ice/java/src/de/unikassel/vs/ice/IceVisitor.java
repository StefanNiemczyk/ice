package de.unikassel.vs.ice;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassExpression;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLDataProperty;
import org.semanticweb.owlapi.model.OWLObjectProperty;
import org.semanticweb.owlapi.model.OWLObjectVisitor;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.reasoner.OWLReasoner;

public abstract class IceVisitor extends IceOntologyInterface implements OWLObjectVisitor {

	// public static final String IRI_PREFIX =
	// "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#";
	protected final IceOntologyInterface ioi;

	protected final OWLClass node;
	protected final OWLClass computationNode;
	protected final OWLClass sourceNode;
	protected final OWLClass iro;
	protected final OWLClass mapNode;
	protected final OWLClass requiredStream;
	protected final OWLClass requiredMap;
	protected final OWLClass system;
	protected final OWLClass entityType;
	protected final OWLClass entityScope;
	protected final OWLClass representation;
	protected final OWLClass valueScope;
	protected final OWLClass stream;
	protected final OWLClass namedStream;
	protected final OWLClass namedMap;
	protected final OWLClass groundingOWLClass;

	protected final OWLObjectProperty hasInput;
	protected final OWLObjectProperty hasRelatedInput;
	protected final OWLObjectProperty hasOutput;
	protected final OWLObjectProperty hasOutputMap;
	protected final OWLObjectProperty isSystemOf;
	protected final OWLObjectProperty isStreamOf;
	protected final OWLObjectProperty isMapOf;
	protected final OWLObjectProperty isGroundingOf;
	protected final OWLObjectProperty onlyEntity;
	protected final OWLObjectProperty hasRepresentation;
	protected final OWLObjectProperty hasScope;
	protected final OWLObjectProperty hasDimension;
	protected final OWLObjectProperty hasRelatedDimension;
	protected final OWLObjectProperty hasUnit;
	protected final OWLObjectProperty hasStreamMetadata;
	protected final OWLObjectProperty hasMetadata;
	protected final OWLObjectProperty improveInformationMetadata;
	protected final OWLObjectProperty impairInformationMetadata;
	protected final OWLObjectProperty hasGrounding;
	protected final OWLObjectProperty aboutEntity;
	protected final OWLObjectProperty aboutRelatedEntity;

	protected final OWLDataProperty hasGroundingValue;
	protected final OWLDataProperty hasMetadataValue;
	protected final OWLDataProperty hasMetadataValue2;
	protected final OWLDataProperty hasConfiguration;

	protected final Set<OWLOntology> ontologies;
	protected final OWLReasoner reasoner;
	protected StringBuffer sb;
	protected List<OWLClass> foundClasses;

	public IceVisitor(final IceOntologyInterface p_ioi, final Set<OWLOntology> p_ontologies,
			final OWLReasoner p_reasoner, final OWLDataFactory p_dataFactory) {
		this.ioi = p_ioi;
		this.ontologies = p_ontologies;
		this.reasoner = p_reasoner;

		this.entityType = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "EntityType"));
		this.entityScope = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "EntityScope"));
		this.representation = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "Representation"));
		this.valueScope = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "ValueScope"));
		this.node = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "Node"));
		this.computationNode = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "ComputationNode"));
		this.sourceNode = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "SourceNode"));
		this.iro = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "IRONode"));
		this.mapNode = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "MapNode"));
		this.requiredStream = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "RequiredStream"));
		this.requiredMap = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "RequiredMap"));
		this.system = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "System"));
		this.stream = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "Stream"));
		this.namedStream = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "NamedStream"));
		this.namedMap = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "NamedMap"));
		this.groundingOWLClass = p_dataFactory.getOWLClass(IRI.create(ICE_IRI_PREFIX + "Grounding"));

		this.hasInput = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasInput"));
		this.hasRelatedInput = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasRelatedInput"));
		this.hasOutput = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasOutput"));
		this.hasOutputMap = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "hasOutputMap"));
		this.isSystemOf = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "isSystemOf"));
		this.isStreamOf = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "isStreamOf"));
		this.isMapOf = p_dataFactory.getOWLObjectProperty(IRI.create(ICE_IRI_PREFIX + "isMapOf"));
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

		this.sb = new StringBuffer();
		this.foundClasses = new ArrayList<OWLClass>();
	}

	protected boolean isSubClassOf(final OWLClassExpression p_child, final OWLClass p_parent) {
		// owlapi 4.0 remove this
		if (p_child.isAnonymous()) {
			return false;
		}

		if (p_child.equals(p_parent))
			return true;

		Set<OWLClass> superClasses = reasoner.getSuperClasses(p_child, true).getFlattened();

		for (OWLClass c : superClasses) {
			if (c.equals(p_parent))
				return true;

			if (this.isSubClassOf(c, p_parent))
				return true;
		}

		return false;
	}

	protected Set<OWLClass> getAllLeafs(final OWLClass p_start) {
		Set<OWLClass> subClasses = reasoner.getSubClasses(p_start, true).getFlattened();
		Set<OWLClass> results = new HashSet<OWLClass>();

		for (OWLClass c : subClasses) {
			if (c.isOWLNothing()) {
				results.add(p_start);
			}

			results.addAll(this.getAllLeafs(c));
		}

		return results;
	}

	protected String iRIShortName(IRI p_iri) {
		String tmp = p_iri.toString().substring(p_iri.toString().indexOf("#") + 1);
		// owlapi 4.0
		// String tmp =
		// p_iri.getShortForm().substring(p_iri.getShortForm().indexOf("#") +
		// 1);

		return Character.toLowerCase(tmp.charAt(0)) + (tmp.length() > 1 ? tmp.substring(1) : "");
	}

	public String toString() {
		return this.sb.toString();
	}

	protected void log(String p_msg) {
		System.out.println("java     " + p_msg);
	}
}
