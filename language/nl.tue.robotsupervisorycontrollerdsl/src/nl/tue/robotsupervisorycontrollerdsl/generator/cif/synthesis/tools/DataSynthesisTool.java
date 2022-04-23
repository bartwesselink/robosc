package nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.tools;

import nl.tue.robotsupervisorycontrollerdsl.cifwrapper.DataSynthesisWrapper;

public class DataSynthesisTool extends AbstractCifTool<DataSynthesisWrapper> {
	public boolean execute(String input, String output) {
		return this.execute(new String[] { "--gui=off", "-o", output, input });
	}
	
	@Override
	Class<DataSynthesisWrapper> getApplication() {
		return DataSynthesisWrapper.class;
	}
}
