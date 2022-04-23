package nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.tools;

import nl.tue.robotsupervisorycontrollerdsl.cifwrapper.CifToCifWrapper;

public class StateExclusionTool extends AbstractCifTool<CifToCifWrapper> {
	public boolean execute(String input, String output) {
		return this.execute(new String[] { "--gui=off", "-o", output, "-t", "elim-state-evt-excl-invs", input });
	}
	
	@Override
	Class<CifToCifWrapper> getApplication() {
		return CifToCifWrapper.class;
	}
}
