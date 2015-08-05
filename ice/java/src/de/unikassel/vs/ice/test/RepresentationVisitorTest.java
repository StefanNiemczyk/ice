package de.unikassel.vs.ice.test;

import static org.junit.Assert.assertEquals;
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
	public final void testAddRepresentation() throws Exception {
		final String before = oi.readRepresentationsAsCSV();
		oi.addRepresentation("IntegerRep", "CountRep", new String[0]);
		final String after = oi.readRepresentationsAsCSV();
		assertEquals(before, after);
	}

	//
	// @Test
	// public final void testDelimInValue() throws Exception {
	// final String delim = RepresentationIndividual.DELIM_STR;
	// final String escapedDelim = "\\" + delim;
	// final String dataString = "This is a bad " + delim + " string";
	// final String reprString = "StringRep";
	// final int repNum = Representation.valueOf(reprString).ordinal();
	// final String expectedLine = repNum + delim + dataString.replace(delim,
	// escapedDelim);
	// final RepresentationIndividual ind = new
	// RepresentationIndividual(reprString, dataString);
	// final String line = ind.toString();
	//
	// assertEquals("Expected line has espaced delim character", expectedLine,
	// line);
	// }
}