package nl.tue.robotsupervisorycontrollerdsl.tests.generator.common.naming

import com.google.inject.Inject
import org.eclipse.xtext.testing.InjectWith
import org.eclipse.xtext.testing.extensions.InjectionExtension
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.^extension.ExtendWith
import nl.tue.robotsupervisorycontrollerdsl.tests.RobotSupervisoryControllerDSLInjectorProvider
import org.junit.jupiter.api.Assertions
import nl.tue.robotsupervisorycontrollerdsl.tests.helper.ParseHelper
import java.util.Random
import nl.tue.robotsupervisorycontrollerdsl.generator.common.naming.TauTransitionNamer

@ExtendWith(InjectionExtension)
@InjectWith(RobotSupervisoryControllerDSLInjectorProvider)
class TauTransitionNamerTest {
	@Inject ParseHelper parseHelper

	@Test
	def void checkSimpleNames() {
		val random = new Random(1L);
		
		val namer = new TauTransitionNamer()
		val transition = parseHelper.parseTauTransition("transition goto state if true")
		
		val name = namer.getName(transition, random)
		
		Assertions.assertEquals("pXSJXQ4EAASPP", name)
	}

	@Test
	def void checkMultipleRuns() {
		val namer = new TauTransitionNamer()
		val transition = parseHelper.parseTauTransition("transition goto state if true")
		
		val nameOne = namer.getName(transition)
		val nameTwo = namer.getName(transition)

		Assertions.assertEquals(nameOne, nameTwo)
	}
}
