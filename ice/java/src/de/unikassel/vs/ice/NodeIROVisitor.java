package de.unikassel.vs.ice;

import java.util.ArrayList;
import java.util.Collection;
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

public class NodeIROVisitor extends IceVisitor {

	private static final String IRO_PLACEHOLDER = "$IRO";

	enum Type {
		NODE, IRO
	};

	private Type currentType;
	private OWLClass lastIRO;
	private OWLClass iroScope;
	private OWLClass iroRelatedScope;
	private OWLClass lastOnlyEntity;
	private OWLClass currentScope;
	private OWLClass currentRepresentation;
	private OWLNamedIndividual currentSystem;
	private String elementString;
	private boolean found;

	private OWLNamedIndividual grounding;

	public NodeIROVisitor(final Set<OWLOntology> p_ontologies, final OWLReasoner p_reasoner,
			final OWLDataFactory p_dataFactory) {
		super(p_ontologies, p_reasoner, p_dataFactory);
	}

	public void start() {
		Set<OWLNamedIndividual> systems = this.reasoner.getInstances(this.system, true).getFlattened();

		for (OWLNamedIndividual system : systems) {
			System.out.println("Creating IROs for system " + system);

			this.currentSystem = system;

			Set<OWLNamedIndividual> groundings = this.reasoner.getObjectPropertyValues(system, isSystemOf)
					.getFlattened();

			for (OWLNamedIndividual grounding : groundings) {
				System.out.println(String.format("Checking grounding %s in system %s", grounding, system));
				this.grounding = grounding;

				this.found = false;
				// check types
				Collection<OWLClassExpression> types = new ArrayList<OWLClassExpression>();

				for (OWLOntology ont : this.ontologies)
					types.addAll(grounding.getTypes(ont));
				// OWL API 4.0
				// types.addAll(EntitySearcher.getTypes(grounding, ont));

				for (OWLClassExpression type : types) {
					type.accept(this);
				}

				// check metadata
				Set<OWLNamedIndividual> metadatas = this.reasoner.getObjectPropertyValues(grounding, this.hasMetadata)
						.getFlattened();

				for (OWLNamedIndividual metadata : metadatas) {
					metadata.accept(this);
				}

				sb.append("\n");
			}
		}
	}

	@Override
	public void visit(OWLNamedIndividual individual) {
		Set<OWLLiteral> literals = this.reasoner.getDataPropertyValues(individual, this.hasMetadataValue);

		if (literals.size() != 1) {
			System.out.println("Wrong size of hasMetadataValue properties " + literals.size() + " " + individual
					+ " will be skipped.");
			return;
		}

		String value = literals.iterator().next().getLiteral();

		Set<OWLNamedIndividual> groundings = this.reasoner.getObjectPropertyValues(individual, this.hasGrounding)
				.getFlattened();

		if (groundings.size() != 1) {
			System.out.println("Wrong size of hasGrounding properties " + groundings.size() + " " + individual
					+ " will be skipped.");
		} else {
			OWLNamedIndividual ind = groundings.iterator().next();

			Collection<OWLClassExpression> types = new ArrayList<OWLClassExpression>();

			for (OWLOntology ont : this.ontologies)
				types.addAll(ind.getTypes(ont));
			// OWL API 4.0
			// types.addAll(EntitySearcher.getTypes(ind, ont));

			int count = 0;

			for (OWLClassExpression type : types) {
				if (this.isSubClassOf(type, this.aSPGrounding))
					++count;
			}

			if (count != 1) {
				System.out.println("Individual " + individual + " is not a ASP Grounding and will be skipped.");
				return;
			}

			literals = this.reasoner.getDataPropertyValues(ind, this.hasASPGrounding);

			for (OWLLiteral lit : literals) {
				String pattern = this.replace(lit.getLiteral());

				pattern = pattern.replace("value1", value);

				sb.append(pattern + "\n");
			}
		}
	}

	@Override
	public void visit(OWLClass ce) {
		// if (foundClasses.contains(ce))
		// return;
		//
		// foundClasses.add(ce);

		OWLClass lastNode = this.lastIRO;
		OWLClass lastOnlyEntity = this.lastOnlyEntity;
		List<OWLSubClassOfAxiom> others = new ArrayList<OWLSubClassOfAxiom>();
		boolean iro = false;
		for (OWLOntology ont : this.ontologies) {
			for (OWLSubClassOfAxiom ax : ont.getSubClassAxiomsForSubClass(ce)) {
				if (this.isSubClassOf(ax.getSuperClass(), this.node)) {
					if (found)
						continue;

					this.found = true;
					this.currentType = Type.NODE;
					this.lastIRO = ce;

					this.sb.append("#program ");
					this.sb.append(this.iRIShortName(this.grounding.getIRI()));
					this.sb.append(".\n");

					// #external nodeTemplate(system1,node1,any).
					StringBuffer sb = new StringBuffer();
					sb.append("nodeTemplate(");
					sb.append(this.iRIShortName(this.currentSystem.getIRI()));
					sb.append(",");
					sb.append(this.iRIShortName(this.grounding.getIRI()));
					sb.append(",");
					sb.append((this.lastOnlyEntity != null) ? this.iRIShortName(this.lastOnlyEntity.getIRI()) : "any");
					sb.append(").\n");

					this.elementString = sb.toString();
					this.sb.append("#external ");
					this.sb.append(this.elementString);
				} else if (this.isSubClassOf(ax.getSuperClass(), this.iro)) {
					if (found)
						continue;

					this.found = true;
					this.currentType = Type.IRO;
					this.lastIRO = ce;
					iro = true;

					this.sb.append("#program ");
					this.sb.append(this.iRIShortName(this.grounding.getIRI()));
					this.sb.append(".\n");
				} else if (this.isSubClassOf(ax.getSuperClass(), this.entityScope)) {
					this.currentScope = ce;
				} else {
					// System.out.println(ax);
					others.add(ax);
				}
			}
		}

		for (OWLSubClassOfAxiom ax : others) {
			// others
			ax.getSuperClass().accept(this);
			// System.out.println(ax.getSuperClass());
		}

		if (iro) {
			// #external iro(system1,coords2Wgs84,any,position).
			StringBuffer sb = new StringBuffer();
			sb.append("iro(");
			sb.append(this.iRIShortName(this.system.getIRI()));
			sb.append(",");
			sb.append(this.iRIShortName(this.grounding.getIRI()));
			sb.append(",");
			sb.append((this.lastOnlyEntity != null) ? this.iRIShortName(this.lastOnlyEntity.getIRI()) : "any");
			sb.append(",");
			sb.append(this.iRIShortName(this.iroScope.getIRI()));

			if (this.iroRelatedScope != null) {
				sb.append((this.lastOnlyEntity != null) ? this.iRIShortName(this.lastOnlyEntity.getIRI()) : "any");
				sb.append(",");
				sb.append(this.iRIShortName(this.iroRelatedScope.getIRI()));
			}

			sb.append(").\n");

			this.elementString = sb.toString();
			this.sb.append("#external ");
			this.sb.append(this.elementString);

			int index;
			while ((index = this.sb.indexOf(IRO_PLACEHOLDER)) >= 0) {
				this.sb.replace(index, index + IRO_PLACEHOLDER.length(), this.elementString);
			}

			this.iroScope = null;
			this.iroRelatedScope = null;
		}

		this.lastIRO = lastNode;
		this.lastOnlyEntity = lastOnlyEntity;
	}

	@Override
	public void visit(OWLObjectExactCardinality ce) {
		int cardinality = ce.getCardinality();
		if (ce.getProperty().equals(hasInput)) {
			// input(system1,node1,scope1,rep1,1,1) :-
			// nodeTemplate(system1,node1,any).

			String pattern = "";

			switch (this.currentType) {
			case NODE:
				pattern = "input(%s,%s,%s,%s," + cardinality + "," + cardinality + ") :- " + this.elementString;
				this.printNodeStreamRelation(ce.getFiller(), pattern);
				break;
			case IRO:
				pattern = "inputIRO(%s,%s,%s,%s," + cardinality + "," + cardinality + ") :- " + IRO_PLACEHOLDER;
				this.printNodeStreamRelation(ce.getFiller(), pattern);
				this.iroScope = this.currentScope;
				System.out.println(currentScope);
				break;
			default:
				// TODO
			}
		} else if (ce.getProperty().equals(hasOutput)) {
			// output(system1,node1,scope3,rep1,max,0).

			String pattern = "";

			switch (this.currentType) {
			case NODE:
				pattern = "output(%s,%s,%s,%s,max,0).\n";
				break;
			case IRO:
				pattern = "outputIRO(%s,%s,%s,%s,max,0).\n";
				break;
			default:
				// TODO
			}

			this.printNodeStreamRelation(ce.getFiller(), pattern);
		} else if (ce.getProperty().equals(hasRelatedInput)) {

			String pattern = "";

			switch (this.currentType) {
			case NODE:
				pattern = "input2(%s,%s,%s,%s," + cardinality + "," + cardinality + ") :- " + this.elementString;
				this.printNodeStreamRelation(ce.getFiller(), pattern);
				break;
			case IRO:
				pattern = "inputIRO2(%s,%s,%s,%s," + cardinality + "," + cardinality + ") :- " + IRO_PLACEHOLDER;
				this.printNodeStreamRelation(ce.getFiller(), pattern);
				this.iroRelatedScope = this.currentScope;
				break;
			default:
				// TODO
			}
		} else {
			System.out.println("Unknown OWLObjectExactCardinality " + ce);
		}
	}

	public void visit(OWLObjectSomeValuesFrom ce) {
		if (ce.getProperty().equals(isStreamOf)) {
			if (ce.getFiller().isAnonymous()) {
				ce.getFiller().accept(this);
			} else {
				this.currentScope = ce.getFiller().asOWLClass();
			}
		} else if (ce.getProperty().equals(hasRepresentation)) {
			if (ce.getFiller().isAnonymous()) {
				System.out.println("Anonymous Representation? " + ce.getFiller());
			} else {
				this.currentRepresentation = ce.getFiller().asOWLClass();
			}
		} else if (ce.getProperty().equals(hasInput)) {
			// input(system1,node1,scope1,rep1,1,1) :-
			// nodeTemplate(system1,node1,any).
			this.printNodeStreamRelation(ce.getFiller(), "input(%s,%s,%s,%s," + 1 + "," + 1 + ") :- "
					+ this.elementString + "\n");
		} else if (ce.getProperty().equals(hasOutput)) {
			// output(system1,node1,scope3,rep1,max,0).
			this.printNodeStreamRelation(ce.getFiller(), "output(%s,%s,%s,%s,max,0).\n");
		} else if (ce.getProperty().equals(hasStreamMetadata)) {
			// currently ignored
		} else if (ce.getProperty().equals(improveInformationMetadata)) {
			// currently ignored
		} else if (ce.getProperty().equals(impairInformationMetadata)) {
			// currently ignored
		} else {
			System.out.println("Unknown OWLObjectSomeValuesFrom " + ce);
		}
	}

	@Override
	public void visit(OWLObjectIntersectionOf ce) {
		TreeSet<OWLClassExpression> treeSet = new TreeSet<OWLClassExpression>(ce.getOperands());
		for (OWLClassExpression exp : treeSet) {
			exp.accept(this);
		}
	}

	public void printNodeStreamRelation(OWLClassExpression p_ce, String p_pattern) {
		this.currentRepresentation = null;
		this.currentScope = null;

		if (p_ce.isAnonymous()) {
			p_ce.accept(this);
		} else {
			OWLClass c = p_ce.asOWLClass();

			if (this.isSubClassOf(c, this.namedStream)) {
				for (OWLOntology ont : this.ontologies) {
					for (OWLSubClassOfAxiom ax : ont.getSubClassAxiomsForSubClass(c)) {
						ax.getSuperClass().accept(this);
					}
				}
			}
		}

		if (this.currentScope != null) {
			if (this.currentRepresentation != null) {
				sb.append(String.format(p_pattern, this.iRIShortName(this.currentSystem.getIRI()),
						this.iRIShortName(this.lastIRO.getIRI()), this.iRIShortName(this.currentScope.getIRI()),
						this.iRIShortName(this.currentRepresentation.getIRI())));
			} else {
				// TODO
				System.out.println("No representation for scope " + this.currentScope + " skipping " + p_pattern);
			}
		}
	}

	private String replace(String p_string) {
		p_string = p_string.replace("system", this.iRIShortName(this.currentSystem.getIRI()));
		return p_string.replace("node", this.iRIShortName(this.grounding.getIRI()));
	}

	// Generic Stuff, which I ignore

	@Override
	public void visit(OWLObjectProperty property) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDeclarationAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDatatypeDefinitionAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLAnnotationAssertionAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLSubAnnotationPropertyOfAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLAnnotationPropertyDomainAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLAnnotationPropertyRangeAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLSubClassOfAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLNegativeObjectPropertyAssertionAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLAsymmetricObjectPropertyAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLReflexiveObjectPropertyAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDisjointClassesAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDataPropertyDomainAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLObjectPropertyDomainAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLEquivalentObjectPropertiesAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLNegativeDataPropertyAssertionAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDifferentIndividualsAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDisjointDataPropertiesAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDisjointObjectPropertiesAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLObjectPropertyRangeAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLObjectPropertyAssertionAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLFunctionalObjectPropertyAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLSubObjectPropertyOfAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDisjointUnionAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLSymmetricObjectPropertyAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDataPropertyRangeAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLFunctionalDataPropertyAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLEquivalentDataPropertiesAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLClassAssertionAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLEquivalentClassesAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLDataPropertyAssertionAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLTransitiveObjectPropertyAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLIrreflexiveObjectPropertyAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLSubDataPropertyOfAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLInverseFunctionalObjectPropertyAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLSameIndividualAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLSubPropertyChainOfAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLInverseObjectPropertiesAxiom axiom) {
		// TODO Auto-generated method stub

	}

	@Override
	public void visit(OWLHasKeyAxiom axiom) {
		// TODO Auto-generated method stub

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
