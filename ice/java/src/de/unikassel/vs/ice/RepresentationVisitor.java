package de.unikassel.vs.ice;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
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
import org.semanticweb.owlapi.model.OWLPropertyExpression;
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

	private final Stack<String> repStack = new Stack<String>();
	private final Stack<String> dimStack = new Stack<String>();
	private final boolean DEBUG = false;
	private final List<Triple> triples = new ArrayList<Triple>();
	private final Stack<String> relations = new Stack<String>();

	class Triple {
		public Triple(String rep, String dim, String repDim) {
			this.representation = rep;
			this.dimension = dim;
			this.dimensionRep = repDim;
		}

		public String representation;
		public String dimension;
		public String dimensionRep;
	}

	public RepresentationVisitor(final IceOntologyInterface ioi, final Set<OWLOntology> ontologies,
			final OWLReasoner reasoner, final IceIris iceIris) {
		super(ioi, ontologies, reasoner, iceIris);
	}

	public void reset() {
		this.repStack.clear();
		this.dimStack.clear();
		this.triples.clear();
		this.relations.clear();
	}

	private void debuglog(String str) {
		if (DEBUG)
			System.err.println(str);
	}

	public int addToStringBuilder(StringBuilder sb) {
		for (Triple triple : this.triples) {
			sb.append(triple.representation + ";" + triple.dimension + ";" + triple.dimensionRep + "\n");
		}

		return this.triples.size();
	}

	public Set<OWLClass> toClassSet(Set<OWLClassExpression> inSet) {
		HashSet<OWLClass> set = new HashSet<OWLClass>();

		for (OWLClassExpression ce : inSet) {
			if (ce.getClassExpressionType() == ClassExpressionType.OWL_CLASS)
				set.add(ce.asOWLClass());
		}

		return set;
	}

	@Override
	public final void visit(final OWLClass cl) {
		debuglog("VISITING: " + iRIShortName(cl.getIRI()));

		List<OWLSubClassOfAxiom> axioms = new ArrayList<OWLSubClassOfAxiom>();
		boolean rep = false;
		boolean dim = false;

		for (OWLOntology ont : ontologies) {
			for (OWLSubClassOfAxiom ax : ont.getSubClassAxiomsForSubClass(cl)) {

				if (this.isSubClassOf(ax.getSuperClass(), this.ii.representation)) {
					rep = true;
				} else if (this.isSubClassOf(ax.getSuperClass(), this.ii.scope)) {
					dim = true;
				} else {
					axioms.add(ax);
				}

			}
		}

		if (rep) {
			if (dimStack.isEmpty() == false && repStack.isEmpty() == false) {
				Triple t = new Triple(repStack.peek(), dimStack.peek(), iRIShortName(cl.getIRI()));
				this.triples.add(t);
			}

			repStack.push(iRIShortName(cl.getIRI()));
		} else if (dim) {
			dimStack.push(iRIShortName(cl.getIRI()));
		}

		for (OWLSubClassOfAxiom ax : axioms) {
			ax.getSuperClass().accept(this);
		}

		if (rep) {
			repStack.pop();
		} else if (dim) {
			dimStack.pop();
		}
	}

	@Override
	public void visit(final OWLObjectSomeValuesFrom ce) {
		debuglog("SOMEVALUESFROM: " + ce.getFiller());

		this.checkProperty(ce.getProperty(), ce.getFiller());
	}

	@Override
	public void visit(final OWLObjectIntersectionOf is) {
		debuglog("    intersection: " + is);

		Set<OWLClassExpression> operands = is.getOperands();

		for (OWLClassExpression exp : operands) {
			exp.accept(this);
		}
	}

	@Override
	public void visit(final OWLObjectExactCardinality ec) {
		this.checkProperty(ec.getProperty(), ec.getFiller());
	}

	public void checkProperty(OWLPropertyExpression p_property, OWLClassExpression p_class) {
		if (p_property == this.ii.hasDimension) {
			if (p_class.isAnonymous()) {
				if (p_class instanceof OWLObjectIntersectionOf) {
					OWLObjectIntersectionOf intersection = (OWLObjectIntersectionOf) p_class;

					TreeSet<OWLClassExpression> treeSet = new TreeSet<OWLClassExpression>(intersection.getOperands());
					OWLClass class1 = null;
					List<OWLClassExpression> exps = new ArrayList<OWLClassExpression>();

					for (OWLClassExpression exp : treeSet) {
						if (exp instanceof OWLClass) {
							class1 = (OWLClass) exp;
						} else {
							exps.add(exp);
						}
					}

					if (class1 != null) {
						dimStack.push(iRIShortName(class1.getIRI()));
						relations.push("dimension");

						for (OWLClassExpression exp : exps) {
							exp.accept(this);
						}

						relations.pop();
						dimStack.pop();
					}
				}

			} else {
				relations.push("dimension");
				p_class.accept(this);
				relations.pop();
			}
		} else if (p_property == this.ii.hasRepresentation) {
			relations.push("representation");
			p_class.accept(this);
			relations.pop();
		}
	}

	@Override
	public void visit(final OWLObjectAllValuesFrom av) {
		// ignore closure axioms
	}

	@Override
	public void visit(OWLSubClassOfAxiom axiom) {
		axiom.getSubClass().accept(this);
		axiom.getSuperClass().accept(this);
	}

	@Override
	public void visit(final OWLObjectHasValue arg0) {
		// System.out.println("HasValue: " + arg0);
	}

	@Override
	public void visit(final OWLObjectMinCardinality arg0) {
		debuglog("    mincard: " + arg0);
		this.checkProperty(arg0.getProperty(), arg0.getFiller());
	}

	@Override
	public void visit(final OWLObjectMaxCardinality arg0) {
		debuglog("    maxcard: " + arg0);
		this.checkProperty(arg0.getProperty(), arg0.getFiller());
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
