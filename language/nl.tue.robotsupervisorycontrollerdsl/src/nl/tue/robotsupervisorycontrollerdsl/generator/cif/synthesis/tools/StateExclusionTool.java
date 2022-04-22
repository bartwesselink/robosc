package nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.tools;

import org.eclipse.escet.cif.cif2cif.app.CifToCifApp;

public class StateExclusionTool extends AbstractCifTool<CifToCifApp> {
	public boolean execute(String input, String output) {
		return this.execute(new String[] { "--gui=off", "-o", output, "-t", "elim-state-evt-excl-invs", input });
	}
	
	@Override
	Class<CifToCifApp> getApplication() {
		return CifToCifApp.class;
	}
}
