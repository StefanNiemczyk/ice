package de.unikassel.vs.ice;

import java.util.ArrayList;
import java.util.Date;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLClassExpression;
import org.semanticweb.owlapi.model.OWLObjectVisitor;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.reasoner.OWLReasoner;

public abstract class IceVisitor implements OWLObjectVisitor {

	protected final IceOntologyInterface ioi;

	protected final Set<OWLOntology> ontologies;
	protected final OWLReasoner reasoner;
	protected IceIris ii;
	protected StringBuffer sb;
	protected List<OWLClass> foundClasses;
	protected LogLevel logLevel;

	public IceVisitor(final IceOntologyInterface p_ioi, final Set<OWLOntology> p_ontologies,
			final OWLReasoner p_reasoner, final IceIris p_iceIris) {
		this.ioi = p_ioi;
		this.ontologies = p_ontologies;
		this.reasoner = p_reasoner;
		this.ii = p_iceIris;

		this.sb = new StringBuffer();
		this.foundClasses = new ArrayList<OWLClass>();
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

	protected boolean isSubClassOf(final OWLClassExpression p_child, final OWLClass p_parent) {
		if (p_child == null)
			return false;

		// owlapi 4.0 remove this
		if (p_child.isAnonymous()) {
			return false;
		}

		if (p_child.equals(p_parent))
			return true;

		Set<OWLClass> superClasses = this.reasoner.getSuperClasses(p_child, true).getFlattened();

		for (OWLClass c : superClasses) {
			if (c.equals(p_parent))
				return true;

			if (this.isSubClassOf(c, p_parent))
				return true;
		}

		return false;
	}

	protected String iRIShortName(IRI p_iri) {
		return this.ioi.iRIShortName(p_iri);

		// String tmp = p_iri.toString().substring(p_iri.toString().indexOf("#")
		// + 1);
		// owlapi 4.0
		// String tmp =
		// p_iri.getShortForm().substring(p_iri.getShortForm().indexOf("#") +
		// 1);

		// return Character.toLowerCase(tmp.charAt(0)) + (tmp.length() > 1 ?
		// tmp.substring(1) : "");
	}

	public String toString() {
		return this.sb.toString();
	}

	protected void log(LogLevel ll, String msg) {
		final String date = IceOntologyInterface.DATE_FORMAT.format(new Date());
		final String className = this.getClass().getSimpleName();
		final String llstr = ll.toString();
		
		if (ioi.getLogLevel() >= ll.getValue()) 
			System.out.printf("[JAVA][%s][%s][%s]: %s\n", llstr, date, className, msg);
	}
	
	protected void logError(String msg) {
		log(LogLevel.Error, msg);
	}

	protected void logWarning(String msg) {
		log(LogLevel.Warning, msg);
	}

	protected void logDebug(String msg) {
		log(LogLevel.Debug, msg);
	}
	
	protected void logInfo(String msg) {
		log(LogLevel.Info, msg);
	}
	
}
