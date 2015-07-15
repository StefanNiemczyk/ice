package de.unikassel.vs.ice.test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

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

	@Test
	public final void testMultipleRepresentations() throws Exception {
		String dataString = "987654321";
		String reprString = "IntegerRep";
		final RepresentationIndividual expectedInd1 = new RepresentationIndividual(reprString, dataString);
		assertTrue(oi.addIndividual(dataString, reprString));
		List<String> expectedLines = new ArrayList<>();
		expectedLines.add(expectedInd1.toString() + "\n");

		dataString = "HelloWorlds";
		reprString = "StringRep";
		final RepresentationIndividual expectedInd2 = new RepresentationIndividual(reprString, dataString);
		assertTrue(oi.addIndividual(dataString, reprString));
		expectedLines.add(expectedInd2.toString() + "\n");

		dataString = "3.1415";
		reprString = "DoubleRep";
		final RepresentationIndividual expectedInd3 = new RepresentationIndividual(reprString, dataString);
		assertTrue(oi.addIndividual(dataString, reprString));
		expectedLines.add(expectedInd3.toString() + "\n");

		Collections.sort(expectedLines, new Comparator<String>() {
			public int extractReprNum(final String str) {
				String numStr = str.split(RepresentationIndividual.DELIM_STR)[0];
				return Integer.parseInt(numStr);
			}

			@Override
			public int compare(final String o1, final String o2) {
				final int num1 = extractReprNum(o1);
				final int num2 = extractReprNum(o2);

				return num1 - num2;
			}
		});

		String expectedLinesStr = "";
		for (String line : expectedLines) {
			expectedLinesStr += line;
		}
		expectedLinesStr = expectedLinesStr.substring(0,
				expectedLinesStr.length() - 1); /* Remove last newline */

		String res = oi.readRepresentationsAsCSV();
		assertFalse("Multiple Representations doesnt result in empty string", res.isEmpty());
		assertEquals("Multiple Representations creates correct lines", expectedLinesStr, res);
	}
}