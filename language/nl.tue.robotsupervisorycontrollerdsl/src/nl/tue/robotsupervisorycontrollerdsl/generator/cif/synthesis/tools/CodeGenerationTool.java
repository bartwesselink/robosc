package nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.tools;

import nl.tue.robotsupervisorycontrollerdsl.cifwrapper.CodeGenWrapper;

public class CodeGenerationTool extends AbstractCifTool<CodeGenWrapper> {
	public boolean execute(String input, String output, String prefix) {
		return this.execute(new String[] { "--gui=off", "-o", output, "--target-language=c99",
				"--code-prefix=" + prefix, "--perform-uncontrollable-events=no", input });
	}

	@Override
	Class<CodeGenWrapper> getApplication() {
		return CodeGenWrapper.class;
	}
}