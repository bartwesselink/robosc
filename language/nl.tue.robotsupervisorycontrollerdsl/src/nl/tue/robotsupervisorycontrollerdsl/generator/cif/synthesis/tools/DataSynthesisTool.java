package nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.tools;

import org.eclipse.escet.cif.datasynth.CifDataSynthesisApp;

public class DataSynthesisTool extends AbstractCifTool<CifDataSynthesisApp> {
	public boolean execute(String input, String output) {
		return this.execute(new String[] { "--gui=off", "-o", output, input });
	}
	
	@Override
	Class<CifDataSynthesisApp> getApplication() {
		return CifDataSynthesisApp.class;
	}
}
