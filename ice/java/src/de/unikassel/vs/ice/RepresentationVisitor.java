package de.unikassel.vs.ice;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Stack;
import java.util.TreeSet;

import org.semanticweb.owlapi.model.ClassExpressionType;
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

	private final HashMap<String, HashSet<String>> representations = new HashMap<String, HashSet<String>>();
	private final Stack<String> repStack = new Stack<String>();
	private final boolean DEBUG = false;

	public RepresentationVisitor(final IceOntologyInterface ioi,
			final Set<OWLOntology> ontologies, final OWLReasoner reasoner,
			final IceIris iceIris) {
		super(ioi, ontologies, reasoner, iceIris);
	}

	private void debuglog(String str) {
		if (DEBUG)
			System.err.println(str);
	}

	@Override
	public String toString() {
		String str = "";

		for (Map.Entry<String, HashSet<String>> entry : representations
				.entrySet()) {
			final String key = entry.getKey();
			final String values = entry.getValue().toString();

			System.out.println(key + ": " + values);
		}

		return str;
	}

	private void addRep(String key, String val) {
		if (key == null || val == null)
			return;

		if (!representations.containsKey(key))
			representations.put(key, new HashSet<String>());

		representations.get(key).add(val);
	}

	public Set<OWLClass> toClassSet(Set<OWLClassExpression> inSet) {
		HashSet<OWLClass> set = new HashSet<OWLClass>();

		for (OWLClassExpression ce : inSet) {
			if (ce.getClassExpressionType() == ClassExpressionType.OWL_CLASS)
				set.add(ce.asOWLClass());
		}

		return set;
	}

	private final void visitClassExpression(OWLClassExpression ex) {
		ClassExpressionType fillerType = ex.getClassExpressionType();
		debuglog("FILLER_TYPE: " + fillerType);

		switch (fillerType) {
		case OWL_CLASS:
			OWLClass cl = ex.asOWLClass();
			String repName = iRIShortName(cl.getIRI());
			addRep(repStack.peek(), repName);
			if (!foundClasses.contains(cl)) {
				cl.accept(this);
			}
			break;

		case OBJECT_SOME_VALUES_FROM: // FALLTHROUGH
		case OBJECT_EXACT_CARDINALITY:
		case OBJECT_INTERSECTION_OF:
		//case OBJECT_HAS_VALUE: // TODO
			ex.accept(this);
			break;

		default:
			break;
		}

	}

	// //////////////
	// TODO: Refactor? Needed?
	private Representation addSubclasses(final OWLClass root) {
		final Set<OWLClass> subClasses = toClassSet(root
				.getSubClasses(ontologies));

		for (OWLClass cl : subClasses) {
			addRep(iRIShortName(root.getIRI()), iRIShortName(cl.getIRI()));
		}

		return null;
	}

	// //////////////

	@Override
	public final void visit(final OWLClass cl) {
		debuglog("VISITING: " + iRIShortName(cl.getIRI()));
		if (foundClasses.contains(cl))
			return;
		foundClasses.add(cl);

		// TODO: Needed or handled by subclassaxioms?
		addSubclasses(cl);

		for (OWLOntology ont : ontologies) {
			for (OWLSubClassOfAxiom ax : ont.getSubClassAxiomsForSubClass(cl)) {
				String currentRep = iRIShortName(ax.getSubClass().asOWLClass()
						.getIRI());
				repStack.push(currentRep);
				debuglog("SubClassOf: " + currentRep);
				ax.getSuperClass().accept(this);
				repStack.pop();
			}
		}
	}

	@Override
	public void visit(final OWLObjectSomeValuesFrom ce) {
		debuglog("SOMEVALUESFROM: " + ce.getFiller());
		visitClassExpression(ce.getFiller());

	}

	@Override
	public void visit(final OWLObjectIntersectionOf is) {
		debuglog("    intersection: " + is);

		Set<OWLClassExpression> operands = is.getOperands();

		for (OWLClassExpression exp : operands) {
			debuglog("\nCURRENT: " + repStack.peek());
			debuglog("EXTYPE: " + exp.getClassExpressionType());
			debuglog("EX: " + exp + "\n");
			visitClassExpression(exp);
		}
	}

	@Override
	public void visit(final OWLObjectExactCardinality ec) {
		OWLClassExpression filler = ec.getFiller();
		visitClassExpression(filler);
	}

	@Override
	public void visit(final OWLObjectAllValuesFrom av) {
		OWLClassExpression filler = av.getFiller();
		visitClassExpression(filler);
	}

	@Override
	public void visit(final OWLObjectMinCardinality arg0) {
		debuglog("    mincard: " + arg0);
	}

	@Override
	public void visit(final OWLObjectMaxCardinality arg0) {
		debuglog("    maxcard: " + arg0);
	}

	/* Ignored visit implementations */

	@Override
	public void visit(final OWLSubDataPropertyOfAxiom arg0) {
		debuglog("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLOntology arg0) {
		debuglog("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDeclarationAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLSubClassOfAxiom ce) {
		System.out.println("    arg: " + ce);
	}

	@Override
	public void visit(final OWLNegativeObjectPropertyAssertionAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLAsymmetricObjectPropertyAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLReflexiveObjectPropertyAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDisjointClassesAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDataPropertyDomainAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLObjectPropertyDomainAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLEquivalentObjectPropertiesAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLNegativeDataPropertyAssertionAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDifferentIndividualsAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDisjointDataPropertiesAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDisjointObjectPropertiesAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLObjectPropertyRangeAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLObjectPropertyAssertionAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLFunctionalObjectPropertyAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLSubObjectPropertyOfAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDisjointUnionAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLSymmetricObjectPropertyAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDataPropertyRangeAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLFunctionalDataPropertyAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLEquivalentDataPropertiesAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLClassAssertionAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLEquivalentClassesAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDataPropertyAssertionAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLTransitiveObjectPropertyAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLIrreflexiveObjectPropertyAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLInverseFunctionalObjectPropertyAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLSameIndividualAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLSubPropertyChainOfAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLInverseObjectPropertiesAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLHasKeyAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDatatypeDefinitionAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final SWRLRule arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLAnnotationAssertionAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLSubAnnotationPropertyOfAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLAnnotationPropertyDomainAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLAnnotationPropertyRangeAxiom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLObjectUnionOf arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLObjectComplementOf arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLObjectHasValue arg0) {
		System.out.println("HasValue: " + arg0);
	}

	@Override
	public void visit(final OWLObjectHasSelf arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLObjectOneOf arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDataSomeValuesFrom arg0) {
		System.out.println("DataValues: " + arg0);
	}

	@Override
	public void visit(final OWLDataAllValuesFrom arg0) {
		System.out.println("AllDataValues: " + arg0);
	}

	@Override
	public void visit(final OWLDataHasValue arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDataMinCardinality arg0) {
		System.out.println("Min: " + arg0);
	}

	@Override
	public void visit(final OWLDataExactCardinality arg0) {
		System.out.println("Exact: " + arg0);
	}

	@Override
	public void visit(final OWLDataMaxCardinality arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLLiteral arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLFacetRestriction arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDatatype arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDataOneOf arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDataComplementOf arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDataIntersectionOf arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDataUnionOf arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDatatypeRestriction arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLObjectProperty arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLObjectInverseOf arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLDataProperty arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLNamedIndividual arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLAnonymousIndividual arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final IRI arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLAnnotation arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final SWRLClassAtom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final SWRLDataRangeAtom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final SWRLObjectPropertyAtom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final SWRLDataPropertyAtom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final SWRLBuiltInAtom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final SWRLIndividualArgument arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final SWRLLiteralArgument arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final SWRLSameIndividualAtom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final SWRLDifferentIndividualsAtom arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final OWLAnnotationProperty arg0) {
		System.out.println("    arg: " + arg0);
	}

	@Override
	public void visit(final SWRLVariable arg0) {
		System.out.println("    arg: " + arg0);
	}

}
