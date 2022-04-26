package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.engine

import javax.inject.Singleton
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.CifSynthesisTool

@Singleton
class ShuffleHelperGenerator {
	def generateShuffleFunction()'''
void shuffle_events(«CifSynthesisTool.codePrefix»_Event_ *x, size_t n)
{
    if (n > 1) {
        srand(time(NULL));

        for (unsigned i = 0; i < n-1; ++i)
        {
            unsigned j = rand() % (n-i) + i;
            «CifSynthesisTool.codePrefix»_Event_ temp = x[i];
            x[i] = x[j];
            x[j] = temp;
        }
    }
}
	'''
}