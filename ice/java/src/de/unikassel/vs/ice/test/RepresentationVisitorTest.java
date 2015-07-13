package de.unikassel.vs.ice.test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;
import de.unikassel.vs.ice.IceOntologyInterface;
import de.unikassel.vs.ice.RepresentationIndividual;

public class RepresentationVisitorTest {

	private IceOntologyInterface oi;

	@Before
	public final void setUp() throws Exception {
		oi = new IceOntologyInterface();
		String path = RepresentationVisitorTest.class.getResource("Ice.owl").getPath().replace("/Ice.owl", "");
		oi.addIRIMapper(path);
		assertTrue(oi.addOntologyIRI("http://www.semanticweb.org/sni/ontologies/2013/7/Ice"));
		assertTrue(oi.loadOntologies());
		assertTrue(oi.isConsistent());
	}

	@Test
	public final void testNoRepresentations() throws Exception {
		String res = oi.readRepresentationsAsCSV();
		assertTrue("No Representations are extracted to empty string", res.isEmpty());
	}

	@Test
	public final void testSingleRepresentation() throws Exception {
		final String dataString = "987654321";
		final String reprString = "IntegerRep";
		final RepresentationIndividual expectedInd = new RepresentationIndividual(reprString, dataString);
		final String expectedLine = expectedInd.toString();

		assertTrue(oi.addIndividual(dataString, reprString));

		String res = oi.readRepresentationsAsCSV();
		assertFalse("Single Representation doesnt result in empty string", res.isEmpty());

		assertEquals("Single integer representation creates correct line", expectedLine, res);
	}

}