package de.unikassel.vs.ice;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLAnnotation;
import org.semanticweb.owlapi.model.OWLAnnotationAssertionAxiom;
import org.semanticweb.owlapi.model.OWLAnnotationProperty;
import org.semanticweb.owlapi.model.OWLAnnotationPropertyDomainAxiom;
import org.semanticweb.owlapi.model.OWLAnnotationPropertyRangeAxiom;
import org.semanticweb.owlapi.model.OWLAnonymousIndividual;
import org.semanticweb.owlapi.model.OWLAsymmetricObjectPropertyAxiom;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassAssertionAxiom;
import org.semanticweb.owlapi.model.OWLClassExpression;
import org.semanticweb.owlapi.model.OWLDataAllValuesFrom;
import org.semanticweb.owlapi.model.OWLDataComplementOf;
import org.semanticweb.owlapi.model.OWLDataExactCardinality;
import org.semanticweb.owlapi.model.OWLDataHasValue;
import org.semanticweb.owlapi.model.OWLDataIntersectionOf;
import org.semanticweb.owlapi.model.OWLDataMaxCardinality;
import org.semanticweb.owlapi.model.OWLDataMinCardinality;
import org.semanticweb.owlapi.model.OWLDataOneOf;
import org.semanticweb.owlapi.model.OWLDataProperty;
import org.semanticweb.owlapi.model.OWLDataPropertyAssertionAxiom;
import org.semanticweb.owlapi.model.OWLDataPropertyDomainAxiom;
import org.semanticweb.owlapi.model.OWLDataPropertyRangeAxiom;
import org.semanticweb.owlapi.model.OWLDataSomeValuesFrom;
import org.semanticweb.owlapi.model.OWLDataUnionOf;
import org.semanticweb.owlapi.model.OWLDatatype;
import org.semanticweb.owlapi.model.OWLDatatypeDefinitionAxiom;
import org.semanticweb.owlapi.model.OWLDatatypeRestriction;
import org.semanticweb.owlapi.model.OWLDeclarationAxiom;
import org.semanticweb.owlapi.model.OWLDifferentIndividualsAxiom;
import org.semanticweb.owlapi.model.OWLDisjointClassesAxiom;
import org.semanticweb.owlapi.model.OWLDisjointDataPropertiesAxiom;
import org.semanticweb.owlapi.model.OWLDisjointObjectPropertiesAxiom;
import org.semanticweb.owlapi.model.OWLDisjointUnionAxiom;
import org.semanticweb.owlapi.model.OWLEquivalentClassesAxiom;
import org.semanticweb.owlapi.model.OWLEquivalentDataPropertiesAxiom;
import org.semanticweb.owlapi.model.OWLEquivalentObjectPropertiesAxiom;
import org.semanticweb.owlapi.model.OWLFacetRestriction;
import org.semanticweb.owlapi.model.OWLFunctionalDataPropertyAxiom;
import org.semanticweb.owlapi.model.OWLFunctionalObjectPropertyAxiom;
import org.semanticweb.owlapi.model.OWLHasKeyAxiom;
import org.semanticweb.owlapi.model.OWLInverseFunctionalObjectPropertyAxiom;
import org.semanticweb.owlapi.model.OWLInverseObjectPropertiesAxiom;
import org.semanticweb.owlapi.model.OWLIrreflexiveObjectPropertyAxiom;
import org.semanticweb.owlapi.model.OWLLiteral;
import org.semanticweb.owlapi.model.OWLNamedIndividual;
import org.semanticweb.owlapi.model.OWLNegativeDataPropertyAssertionAxiom;
import org.semanticweb.owlapi.model.OWLNegativeObjectPropertyAssertionAxiom;
import org.semanticweb.owlapi.model.OWLObjectAllValuesFrom;
import org.semanticweb.owlapi.model.OWLObjectComplementOf;
import org.semanticweb.owlapi.model.OWLObjectExactCardinality;
import org.semanticweb.owlapi.model.OWLObjectHasSelf;
import org.semanticweb.owlapi.model.OWLObjectHasValue;
import org.semanticweb.owlapi.model.OWLObjectIntersectionOf;
import org.semanticweb.owlapi.model.OWLObjectInverseOf;
import org.semanticweb.owlapi.model.OWLObjectMaxCardinality;
import org.semanticweb.owlapi.model.OWLObjectMinCardinality;
import org.semanticweb.owlapi.model.OWLObjectOneOf;
import org.semanticweb.owlapi.model.OWLObjectProperty;
import org.semanticweb.owlapi.model.OWLObjectPropertyAssertionAxiom;
import org.semanticweb.owlapi.model.OWLObjectPropertyDomainAxiom;
import org.semanticweb.owlapi.model.OWLObjectPropertyRangeAxiom;
import org.semanticweb.owlapi.model.OWLObjectSomeValuesFrom;
import org.semanticweb.owlapi.model.OWLObjectUnionOf;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLReflexiveObjectPropertyAxiom;
import org.semanticweb.owlapi.model.OWLSameIndividualAxiom;
import org.semanticweb.owlapi.model.OWLSubAnnotationPropertyOfAxiom;
import org.semanticweb.owlapi.model.OWLSubClassOfAxiom;
import org.semanticweb.owlapi.model.OWLSubDataPropertyOfAxiom;
import org.semanticweb.owlapi.model.OWLSubObjectPropertyOfAxiom;
import org.semanticweb.owlapi.model.OWLSubPropertyChainOfAxiom;
import org.semanticweb.owlapi.model.OWLSymmetricObjectPropertyAxiom;
import org.semanticweb.owlapi.model.OWLTransitiveObjectPropertyAxiom;
import org.semanticweb.owlapi.model.SWRLBuiltInAtom;
import org.semanticweb.owlapi.model.SWRLClassAtom;
import org.semanticweb.owlapi.model.SWRLDataPropertyAtom;
import org.semanticweb.owlapi.model.SWRLDataRangeAtom;
import org.semanticweb.owlapi.model.SWRLDifferentIndividualsAtom;
import org.semanticweb.owlapi.model.SWRLIndividualArgument;
import org.semanticweb.owlapi.model.SWRLLiteralArgument;
import org.semanticweb.owlapi.model.SWRLObjectPropertyAtom;
import org.semanticweb.owlapi.model.SWRLRule;
import org.semanticweb.owlapi.model.SWRLSameIndividualAtom;
import org.semanticweb.owlapi.model.SWRLVariable;
import org.semanticweb.owlapi.reasoner.OWLReasoner;

public class RepresentationVisitor extends IceVisitor {

	private final HashMap<String, Representation> representations = new HashMap<String, Representation>();

	public RepresentationVisitor(final IceOntologyInterface ioi, final Set<OWLOntology> ontologies,
			final OWLReasoner reasoner, final IceIris iceIris) {
		super(ioi, ontologies, reasoner, iceIris);
	}

	public static List<Representation> asSortedList(final Set<Representation> set) {
		final List<Representation> list = new ArrayList<Representation>(set);
		Collections.sort(list);
		return list;
	}

	@Override
	public final String toString() {
		String str = "";
		boolean lineAdded = false;

		for (final Representation ri : representations.values()) {
			str += ri.toString() + '\n';
			lineAdded = true;
		}

		if (lineAdded) {
			str = str.substring(0, str.length() - 1); /* Remove last newline */
		}
		return str;
	}

	private String extractRepresentationName(final IRI iri) {
		return iri.toString().substring(iri.toString().indexOf("#") + 1);
	}

	private Representation extractParent(final OWLClass cl) {
		final Set<OWLClassExpression> sces = cl.getSuperClasses(ontologies);

		if (sces.size() != 1) {
			return null;
		}

		OWLClassExpression sce = null;
		for (final OWLClassExpression owlClassExpression : sces) {
			sce = owlClassExpression;
		}

		final Set<OWLClass> scls = sce.getClassesInSignature();
		if (scls.size() != 1) {
			System.err.println("Warning! Can't handle multiple superclasses yet!");
			System.err.println("Class: " + cl);
			return null;
		}

		OWLClass scl = null;
		for (final OWLClass owlClass : scls) {
			scl = owlClass;
		}

		final String name = extractRepresentationName(scl.getIRI());
		Representation rep = representations.get(name);
		if (rep == null) {
			rep = new Representation(extractRepresentationName(scl.getIRI()), extractParent(scl));
		}

		return rep;
	}

	private Representation makeRepresentation(final OWLClass cl) {
		final String name = extractRepresentationName(cl.getIRI());
		final Representation parent = extractParent(cl);

		return new Representation(name, parent);
	}

	@Override
	public final void visit(final OWLClass cl) {
		final Representation r = makeRepresentation(cl);
		if (r == null) {
			return;
		}

		if (!representations.containsKey(r.name)) {
			representations.put(r.name, r);
		}
	}

	public HashMap<String, Representation> getRepresentations() {
		return representations;
	}

	/* Ignored visit implementations */

	@Override
	public void visit(final OWLOntology arg0) {
	}

	@Override
	public void visit(final OWLDeclarationAxiom arg0) {
	}

	@Override
	public void visit(final OWLSubClassOfAxiom arg0) {
	}

	@Override
	public void visit(final OWLNegativeObjectPropertyAssertionAxiom arg0) {
	}

	@Override
	public void visit(final OWLAsymmetricObjectPropertyAxiom arg0) {
	}

	@Override
	public void visit(final OWLReflexiveObjectPropertyAxiom arg0) {
	}

	@Override
	public void visit(final OWLDisjointClassesAxiom arg0) {
	}

	@Override
	public void visit(final OWLDataPropertyDomainAxiom arg0) {
	}

	@Override
	public void visit(final OWLObjectPropertyDomainAxiom arg0) {
	}

	@Override
	public void visit(final OWLEquivalentObjectPropertiesAxiom arg0) {
	}

	@Override
	public void visit(final OWLNegativeDataPropertyAssertionAxiom arg0) {
	}

	@Override
	public void visit(final OWLDifferentIndividualsAxiom arg0) {
	}

	@Override
	public void visit(final OWLDisjointDataPropertiesAxiom arg0) {
	}

	@Override
	public void visit(final OWLDisjointObjectPropertiesAxiom arg0) {
	}

	@Override
	public void visit(final OWLObjectPropertyRangeAxiom arg0) {
	}

	@Override
	public void visit(final OWLObjectPropertyAssertionAxiom arg0) {
	}

	@Override
	public void visit(final OWLFunctionalObjectPropertyAxiom arg0) {
	}

	@Override
	public void visit(final OWLSubObjectPropertyOfAxiom arg0) {
	}

	@Override
	public void visit(final OWLDisjointUnionAxiom arg0) {
	}

	@Override
	public void visit(final OWLSymmetricObjectPropertyAxiom arg0) {
	}

	@Override
	public void visit(final OWLDataPropertyRangeAxiom arg0) {
	}

	@Override
	public void visit(final OWLFunctionalDataPropertyAxiom arg0) {
	}

	@Override
	public void visit(final OWLEquivalentDataPropertiesAxiom arg0) {
	}

	@Override
	public void visit(final OWLClassAssertionAxiom arg0) {
	}

	@Override
	public void visit(final OWLEquivalentClassesAxiom arg0) {
	}

	@Override
	public void visit(final OWLDataPropertyAssertionAxiom arg0) {
	}

	@Override
	public void visit(final OWLTransitiveObjectPropertyAxiom arg0) {
	}

	@Override
	public void visit(final OWLIrreflexiveObjectPropertyAxiom arg0) {
	}

	@Override
	public void visit(final OWLSubDataPropertyOfAxiom arg0) {
	}

	@Override
	public void visit(final OWLInverseFunctionalObjectPropertyAxiom arg0) {
	}

	@Override
	public void visit(final OWLSameIndividualAxiom arg0) {
	}

	@Override
	public void visit(final OWLSubPropertyChainOfAxiom arg0) {
	}

	@Override
	public void visit(final OWLInverseObjectPropertiesAxiom arg0) {
	}

	@Override
	public void visit(final OWLHasKeyAxiom arg0) {
	}

	@Override
	public void visit(final OWLDatatypeDefinitionAxiom arg0) {
	}

	@Override
	public void visit(final SWRLRule arg0) {
	}

	@Override
	public void visit(final OWLAnnotationAssertionAxiom arg0) {
	}

	@Override
	public void visit(final OWLSubAnnotationPropertyOfAxiom arg0) {
	}

	@Override
	public void visit(final OWLAnnotationPropertyDomainAxiom arg0) {
	}

	@Override
	public void visit(final OWLAnnotationPropertyRangeAxiom arg0) {
	}

	@Override
	public void visit(final OWLObjectIntersectionOf arg0) {
	}

	@Override
	public void visit(final OWLObjectUnionOf arg0) {
	}

	@Override
	public void visit(final OWLObjectComplementOf arg0) {
	}

	@Override
	public void visit(final OWLObjectSomeValuesFrom arg0) {
	}

	@Override
	public void visit(final OWLObjectAllValuesFrom arg0) {
	}

	@Override
	public void visit(final OWLObjectHasValue arg0) {
	}

	@Override
	public void visit(final OWLObjectMinCardinality arg0) {
	}

	@Override
	public void visit(final OWLObjectExactCardinality arg0) {
	}

	@Override
	public void visit(final OWLObjectMaxCardinality arg0) {
	}

	@Override
	public void visit(final OWLObjectHasSelf arg0) {
	}

	@Override
	public void visit(final OWLObjectOneOf arg0) {
	}

	@Override
	public void visit(final OWLDataSomeValuesFrom arg0) {
	}

	@Override
	public void visit(final OWLDataAllValuesFrom arg0) {
	}

	@Override
	public void visit(final OWLDataHasValue arg0) {
	}

	@Override
	public void visit(final OWLDataMinCardinality arg0) {
	}

	@Override
	public void visit(final OWLDataExactCardinality arg0) {
	}

	@Override
	public void visit(final OWLDataMaxCardinality arg0) {
	}

	@Override
	public void visit(final OWLLiteral arg0) {
	}

	@Override
	public void visit(final OWLFacetRestriction arg0) {
	}

	@Override
	public void visit(final OWLDatatype arg0) {
	}

	@Override
	public void visit(final OWLDataOneOf arg0) {
	}

	@Override
	public void visit(final OWLDataComplementOf arg0) {
	}

	@Override
	public void visit(final OWLDataIntersectionOf arg0) {
	}

	@Override
	public void visit(final OWLDataUnionOf arg0) {
	}

	@Override
	public void visit(final OWLDatatypeRestriction arg0) {
	}

	@Override
	public void visit(final OWLObjectProperty arg0) {
	}

	@Override
	public void visit(final OWLObjectInverseOf arg0) {
	}

	@Override
	public void visit(final OWLDataProperty arg0) {
	}

	@Override
	public void visit(final OWLNamedIndividual arg0) {
	}

	@Override
	public void visit(final OWLAnonymousIndividual arg0) {
	}

	@Override
	public void visit(final IRI arg0) {
	}

	@Override
	public void visit(final OWLAnnotation arg0) {
	}

	@Override
	public void visit(final SWRLClassAtom arg0) {
	}

	@Override
	public void visit(final SWRLDataRangeAtom arg0) {
	}

	@Override
	public void visit(final SWRLObjectPropertyAtom arg0) {
	}

	@Override
	public void visit(final SWRLDataPropertyAtom arg0) {
	}

	@Override
	public void visit(final SWRLBuiltInAtom arg0) {
	}

	@Override
	public void visit(final SWRLIndividualArgument arg0) {
	}

	@Override
	public void visit(final SWRLLiteralArgument arg0) {
	}

	@Override
	public void visit(final SWRLSameIndividualAtom arg0) {
	}

	@Override
	public void visit(final SWRLDifferentIndividualsAtom arg0) {
	}

	@Override
	public void visit(final OWLAnnotationProperty arg0) {
	}

	@Override
	public void visit(final SWRLVariable arg0) {
	}

}
