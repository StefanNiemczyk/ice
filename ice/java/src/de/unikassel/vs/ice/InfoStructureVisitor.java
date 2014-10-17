package de.unikassel.vs.ice;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

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
import org.semanticweb.owlapi.model.OWLDataFactory;
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

public class InfoStructureVisitor extends IceVisitor {

	private Set<String> anonymiousThings;

	private OWLClass lastEntity;
	private OWLClass lastScope;
	private OWLClass lastEntityScope;
	private OWLClass lastValueScope;
	private OWLClass lastRepresentation;

	public InfoStructureVisitor(final Set<OWLOntology> p_ontologies, final OWLReasoner p_reasoner,
			final OWLDataFactory p_dataFactory) {
		super(p_ontologies, p_reasoner, p_dataFactory);

		this.anonymiousThings = new HashSet<String>();
	}

	@Override
	public void visit(OWLClass ce) {
		if (foundClasses.contains(ce))
			return;

		foundClasses.add(ce);

		OWLClass lastEntity = this.lastEntity;
		OWLClass lastScope = this.lastScope;
		OWLClass lastEntityScope = this.lastEntityScope;
		OWLClass lastValueScope = this.lastValueScope;
		OWLClass lastRepresentation = this.lastRepresentation;
		List<OWLSubClassOfAxiom> others = new ArrayList<OWLSubClassOfAxiom>();

		for (OWLOntology ont : this.ontologies) {

			for (OWLSubClassOfAxiom ax : ont.getSubClassAxiomsForSubClass(ce)) {
				if (this.isSubClassOf(ax.getSuperClass(), entityType)) {
					sb.append("entityType(");
					sb.append(this.iRIShortName(ce.getIRI()));
					this.lastEntity = ce;
					sb.append(").\n");
				} else if (this.isSubClassOf(ax.getSuperClass(), entityScope)) {
					sb.append("scope(");
					sb.append(this.iRIShortName(ce.getIRI()));
					this.lastEntityScope = ce;
					this.lastScope = ce;
					sb.append(").\n");
				} else if (this.isSubClassOf(ax.getSuperClass(), valueScope)) {
					sb.append("valueScope(");
					sb.append(this.iRIShortName(ce.getIRI()));
					this.lastValueScope = ce;
					this.lastScope = ce;
					sb.append(").\n");
				} else if (this.isSubClassOf(ax.getSuperClass(), representation)) {
					sb.append("representation(");
					sb.append(this.iRIShortName(ce.getIRI()));
					this.lastRepresentation = ce;
					sb.append(").\n");
				} else {
					others.add(ax);
				}
			}
		}

		for (OWLSubClassOfAxiom ax : others) {
			// others
			ax.getSuperClass().accept(this);
			// System.out.println(ax.getSuperClass());
		}

		this.lastEntity = lastEntity;
		this.lastScope = lastScope;
		this.lastEntityScope = lastEntityScope;
		this.lastValueScope = lastValueScope;
		this.lastRepresentation = lastRepresentation;
	}

	@Override
	public void visit(OWLObjectSomeValuesFrom ce) {
		if (ce.getProperty().equals(hasScope)) {
			Set<OWLClass> subs = this.getAllLeafs(ce.getFiller().asOWLClass());

			for (OWLClass c : subs) {
				sb.append("hasScope(");
				sb.append(this.iRIShortName(this.lastEntity.getIRI()));
				sb.append(',');
				sb.append(this.iRIShortName(c.getIRI()));
				sb.append(").\n");

				c.accept(this);
			}
		} else if (ce.getProperty().equals(hasRepresentation)) {
			Set<OWLClass> subs = this.getAllLeafs(ce.getFiller().asOWLClass());

			for (OWLClass c : subs) {
				sb.append("hasRepresentation(");
				sb.append(this.iRIShortName(this.lastScope.getIRI()));
				sb.append(',');
				sb.append(this.iRIShortName(c.getIRI()));
				sb.append(").\n");

				c.accept(this);
			}
		} else if (ce.getProperty().equals(hasDimension)) {
			Set<OWLClass> subs = this.getAllLeafs(ce.getFiller().asOWLClass());

			for (OWLClass c : subs) {
				sb.append("hasDimension(");
				sb.append(this.iRIShortName(this.lastRepresentation.getIRI()));
				sb.append(',');
				sb.append(this.iRIShortName(c.getIRI()));
				sb.append(").\n");

				c.accept(this);
			}
		} else if (ce.getProperty().equals(hasRelatedDimension)) {
			if (ce.getFiller().isAnonymous()) {
				this.createAnonymiousRelatedDimension(ce.getFiller(), hasRepresentation,
						"hasRelatedDimension(%s,%s,%s,%d,%d).\n", 1, 1);
			} else {
				Set<OWLClass> subs = this.getAllLeafs(ce.getFiller().asOWLClass());

				for (OWLClass c : subs) {
					sb.append("hasRelatedDimension(");
					sb.append(this.iRIShortName(this.lastRepresentation.getIRI()));
					sb.append(',');
					sb.append(this.iRIShortName(c.getIRI()));
					sb.append(").\n");

					c.accept(this);
				}
			}
		} else {
			System.out.println("Unknown OWLObjectSomeValuesFrom " + ce);
		}
	}

	@Override
	public void visit(OWLSubClassOfAxiom axiom) {
		sb.append("SubClassOf(");
		axiom.getSubClass().accept(this);
		// insertSpace();
		axiom.getSuperClass().accept(this);
		sb.append(')');
	}

	@Override
	public void visit(OWLObjectExactCardinality ce) {

		if (ce.getProperty().equals(hasDimension)) {
			Set<OWLClass> subs = this.getAllLeafs(ce.getFiller().asOWLClass());

			for (OWLClass c : subs) {
				sb.append("hasDimension(");
				sb.append(this.iRIShortName(this.lastRepresentation.getIRI()));
				sb.append(',');
				sb.append(this.iRIShortName(c.getIRI()));
				sb.append(",");
				sb.append(ce.getCardinality());
				sb.append(",");
				sb.append(ce.getCardinality());
				sb.append(").\n");

				c.accept(this);
			}
		} else if (ce.getProperty().equals(hasRepresentation)) {
			if (ce.getFiller().isAnonymous()) {
				this.createAnonymiousUnit(ce.getFiller(), hasUnit, "hasRepresentation(%s,%s).\n", "hasUnit(%s,%s).\n");
			} else {
				// cardinalities are ignored
				Set<OWLClass> subs = this.getAllLeafs(ce.getFiller().asOWLClass());

				for (OWLClass c : subs) {
					sb.append("hasRepresentation(");
					sb.append(this.iRIShortName(this.lastScope.getIRI()));
					sb.append(',');
					sb.append(this.iRIShortName(c.getIRI()));
					sb.append(").\n");

					c.accept(this);
				}
			}
		} else {
			System.out.println("Unknown OWLObjectExactCardinality " + ce);
		}
	}

	private void createAnonymiousRelatedDimension(OWLClassExpression p_ce, OWLObjectProperty p_property,
			String p_axiom, int min, int max) {
		if (p_ce instanceof OWLObjectIntersectionOf) {
			OWLObjectIntersectionOf intersection = (OWLObjectIntersectionOf) p_ce;

			TreeSet<OWLClassExpression> treeSet = new TreeSet<OWLClassExpression>(intersection.getOperands());
			OWLClass class1 = null;
			String unit = null;
			for (OWLClassExpression exp : treeSet) {
				if (exp instanceof OWLClass) {
					class1 = (OWLClass) exp;
				} else if (exp instanceof OWLObjectHasValue) {
					OWLObjectHasValue ohv = (OWLObjectHasValue) exp;
					if (ohv.getProperty().equals(p_property) && ohv.getFiller().isAnonymous() == false) {
						unit = this.iRIShortName(ohv.getFiller().asOWLNamedIndividual().getIRI());
					}
				} else if (exp instanceof OWLObjectExactCardinality) {
					OWLObjectExactCardinality ohv = (OWLObjectExactCardinality) exp;
					if (ohv.getProperty().equals(p_property) && ohv.getFiller().isAnonymous() == false) {
						unit = this.iRIShortName(ohv.getFiller().asOWLClass().getIRI());
					}
				} else {
					System.out.println(exp);
				}
			}

			if (class1 != null && unit != null) {
				sb.append(String.format(p_axiom, this.iRIShortName(this.lastRepresentation.getIRI()),
						this.iRIShortName(class1.getIRI()), unit, min, max));

				// if (false == this.anonymiousThings.contains(newUnit)) {
				// this.anonymiousThings.add(newUnit);
				//
				// sb.append(String.format(p_create, newUnit, unit));
				// }
			}
		}

	}

	private void createAnonymiousUnit(OWLClassExpression p_ce, OWLObjectProperty p_property, String p_axiom,
			String p_create) {
		if (p_ce instanceof OWLObjectIntersectionOf) {
			OWLObjectIntersectionOf intersection = (OWLObjectIntersectionOf) p_ce;

			TreeSet<OWLClassExpression> treeSet = new TreeSet<OWLClassExpression>(intersection.getOperands());
			OWLClass class1 = null;
			String unit = null;
			for (OWLClassExpression exp : treeSet) {
				if (exp instanceof OWLClass) {
					class1 = (OWLClass) exp;
				} else if (exp instanceof OWLObjectHasValue) {
					OWLObjectHasValue ohv = (OWLObjectHasValue) exp;
					if (ohv.getProperty().equals(p_property) && ohv.getFiller().isAnonymous() == false) {
						unit = this.iRIShortName(ohv.getFiller().asOWLNamedIndividual().getIRI());
					}
				} else if (exp instanceof OWLObjectExactCardinality) {
					OWLObjectExactCardinality ohv = (OWLObjectExactCardinality) exp;
					if (ohv.getProperty().equals(p_property) && ohv.getFiller().isAnonymous() == false) {
						unit = this.iRIShortName(ohv.getFiller().asOWLClass().getIRI());
					}
				} else {
					System.out.println(exp);
				}
			}

			if (class1 != null && unit != null) {
				String newUnit = this.iRIShortName(class1.getIRI()) + '_' + unit;

				sb.append(String.format(p_axiom, this.iRIShortName(this.lastScope.getIRI()), newUnit));

				if (false == this.anonymiousThings.contains(newUnit)) {
					this.anonymiousThings.add(newUnit);

					sb.append(String.format(p_create, newUnit, unit));
				}
			}
		}

	}

	@Override
	public void visit(OWLObjectIntersectionOf ce) {
		TreeSet<OWLClassExpression> treeSet = new TreeSet<OWLClassExpression>(ce.getOperands());
		for (OWLClassExpression exp : treeSet) {
			exp.accept(this);
		}
	}

	// Generic Stuff, which I ignore

	@Override
	public void visit(OWLObjectProperty axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLDeclarationAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLDatatypeDefinitionAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLAnnotationAssertionAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLSubAnnotationPropertyOfAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLAnnotationPropertyDomainAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLAnnotationPropertyRangeAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLNegativeObjectPropertyAssertionAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLAsymmetricObjectPropertyAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLReflexiveObjectPropertyAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLDisjointClassesAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLDataPropertyDomainAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLObjectPropertyDomainAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLEquivalentObjectPropertiesAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLNegativeDataPropertyAssertionAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLDifferentIndividualsAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLDisjointDataPropertiesAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLDisjointObjectPropertiesAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLObjectPropertyRangeAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLObjectPropertyAssertionAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLFunctionalObjectPropertyAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLSubObjectPropertyOfAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLDisjointUnionAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLSymmetricObjectPropertyAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLDataPropertyRangeAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLFunctionalDataPropertyAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLEquivalentDataPropertiesAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLClassAssertionAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLEquivalentClassesAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLDataPropertyAssertionAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLTransitiveObjectPropertyAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLIrreflexiveObjectPropertyAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLSubDataPropertyOfAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLInverseFunctionalObjectPropertyAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLSameIndividualAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLSubPropertyChainOfAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLInverseObjectPropertiesAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(OWLHasKeyAxiom axiom) {
		// TODO Auto-generated method stub

		System.out.println("Unknown " + axiom);
	}

	@Override
	public void visit(SWRLRule rule) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLObjectUnionOf ce) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLObjectComplementOf ce) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLObjectAllValuesFrom ce) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLObjectHasValue ce) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLObjectMinCardinality ce) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLObjectMaxCardinality ce) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLObjectHasSelf ce) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLObjectOneOf ce) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDataSomeValuesFrom ce) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDataAllValuesFrom ce) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDataHasValue ce) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDataMinCardinality ce) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDataExactCardinality ce) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDataMaxCardinality ce) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLLiteral node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLFacetRestriction node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDatatype node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDataOneOf node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDataComplementOf node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDataIntersectionOf node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDataUnionOf node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDatatypeRestriction node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLObjectInverseOf property) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDataProperty property) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLAnnotationProperty property) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLNamedIndividual individual) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLAnonymousIndividual individual) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(IRI iri) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLAnnotation node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(SWRLClassAtom node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(SWRLDataRangeAtom node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(SWRLObjectPropertyAtom node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(SWRLDataPropertyAtom node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(SWRLBuiltInAtom node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(SWRLVariable node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(SWRLIndividualArgument node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(SWRLLiteralArgument node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(SWRLSameIndividualAtom node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(SWRLDifferentIndividualsAtom node) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLOntology ontology) {
		// TODO Auto-generated method stub

	}

}
