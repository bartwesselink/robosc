package nl.tue.robotsupervisorycontrollerdsl.tests.generator.common.naming

import com.google.inject.Inject
import org.eclipse.xtext.testing.InjectWith
import org.eclipse.xtext.testing.extensions.InjectionExtension
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.^extension.ExtendWith
import nl.tue.robotsupervisorycontrollerdsl.tests.RobotSupervisoryControllerDSLInjectorProvider
import org.junit.jupiter.api.Assertions
import nl.tue.robotsupervisorycontrollerdsl.tests.helper.ParseHelper
import nl.tue.robotsupervisorycontrollerdsl.generator.common.naming.ProvideStatementNamer
import java.util.Random

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class ProvideStatementNamerTest {
	@Inject ParseHelper parseHelper

	@Test
	def void checkSimpleNames() {
		val random = new Random(0L);
		
		val namer = new ProvideStatementNamer()
		val provideStatement = parseHelper.parseProvideStatement("provide message with 5 if true")
		
		val name = namer.getName(provideStatement, random)
		
		Assertions.assertEquals("pOGDBZTZLF25H", name)
	}

	@Test
	def void checkMultipleRuns() {
		val namer = new ProvideStatementNamer()
		val provideStatement = parseHelper.parseProvideStatement("provide message with 5 if true")
		
		val nameOne = namer.getName(provideStatement)
		val nameTwo = namer.getName(provideStatement)

		Assertions.assertEquals(nameOne, nameTwo)
	}
}
