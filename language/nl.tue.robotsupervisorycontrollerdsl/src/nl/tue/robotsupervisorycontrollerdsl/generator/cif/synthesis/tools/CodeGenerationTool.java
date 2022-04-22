package nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.tools;

import org.eclipse.escet.cif.codegen.CodeGenApp;

public class CodeGenerationTool extends AbstractCifTool<CodeGenApp> {
	public boolean execute(String input, String output, String prefix) {
		return this.execute(new String[] { "--gui=off", "-o", output, "--target-language=c99",
				"--code-prefix=" + prefix, "--perform-uncontrollable-events=no", input });
	}

	@Override
	Class<CodeGenApp> getApplication() {
		return CodeGenApp.class;
	}
}