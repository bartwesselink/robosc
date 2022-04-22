package nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis;

import java.io.File;

import javax.inject.Inject;
import javax.inject.Singleton;

import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.tools.CodeGenerationTool;
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.tools.DataSynthesisTool;
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.tools.StateExclusionTool;

@Singleton
public class CifSynthesisTool {
	@Inject DataSynthesisTool dataSynthesisTool;
	@Inject StateExclusionTool stateExclusionTool;
	@Inject CodeGenerationTool codeGenerationTool;

	public void applySynthesis(String inputFile) {
		File input = new File(inputFile);
		File directory = input.getParentFile();
		String supervisorFile = directory.getAbsolutePath() + "/supervisor.cif";
		String supervisorConvertedFile = directory.getAbsolutePath() + "/supervisor_converted.cif";
		String codePath = directory.getAbsolutePath() + "/controller/";

		String prefix = "controller";

		dataSynthesisTool.execute(inputFile, supervisorFile);
		stateExclusionTool.execute(supervisorFile, supervisorConvertedFile);
		codeGenerationTool.execute(supervisorConvertedFile, codePath, prefix);
	}
}
