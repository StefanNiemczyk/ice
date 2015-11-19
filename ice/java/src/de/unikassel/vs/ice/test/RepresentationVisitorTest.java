package de.unikassel.vs.ice.test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

import de.unikassel.vs.ice.IceOntologyInterface;

public class RepresentationVisitorTest {
	private IceOntologyInterface oi;

	@Before
	public final void setUp() throws Exception {
		oi = new IceOntologyInterface();
		final String path = RepresentationVisitorTest.class.getResource("Ice.owl").getPath().replace("/Ice.owl", "");
		oi.addIRIMapper(path);
		assertTrue(oi.addOntologyIRI("http://www.semanticweb.org/sni/ontologies/2013/7/Ice"));
		assertTrue(oi.loadOntologies());
		assertTrue(oi.isConsistent());
	}

	@Test
	public final void testDefaultRepresentations() throws Exception {
		final String res = oi.readRepresentationsAsCSV();
		assertFalse(res.isEmpty());
		// TODO: Probably test more?
	}

	@Test
	public final void testAddRepresentationBug() throws Exception {
		final String before = oi.readRepresentationsAsCSV();
		oi.addRepresentation("IntegerRep", "CountRep", new String[0]);
		final String after = oi.readRepresentationsAsCSV();
		// TODO: Fix once bug is fixed
		assertFalse(after.equals(before));
	}
}