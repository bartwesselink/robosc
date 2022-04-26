package nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;

import javax.inject.Inject;
import javax.inject.Singleton;

import org.eclipse.xtext.generator.IFileSystemAccess2;

import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.tools.CodeGenerationTool;
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.tools.DataSynthesisTool;
import nl.tue.robotsupervisorycontrollerdsl.generator.cif.synthesis.tools.StateExclusionTool;

@Singleton
public class CifSynthesisTool {
	@Inject DataSynthesisTool dataSynthesisTool;
	@Inject StateExclusionTool stateExclusionTool;
	@Inject CodeGenerationTool codeGenerationTool;
	public static String codePrefix = "controller";

	public void applySynthesis(String inputFile) {
		File input = new File(inputFile);
		File directory = input.getParentFile();
		String supervisorFile = directory.getAbsolutePath() + "/supervisor.cif";
		String supervisorConvertedFile = directory.getAbsolutePath() + "/supervisor_converted.cif";
		String codePath = directory.getAbsolutePath() + "/controller/";

		dataSynthesisTool.execute(inputFile, supervisorFile);
		stateExclusionTool.execute(supervisorFile, supervisorConvertedFile);
		codeGenerationTool.execute(supervisorConvertedFile, codePath, codePrefix);
	}
	
	public void copyOutputFiles(IFileSystemAccess2 fileSystemAccess, String baseDirectory, String relativeOutputPath) throws IOException {
		String[] files = new String[] { CifSynthesisTool.codePrefix + "_engine.c", CifSynthesisTool.codePrefix + "_engine.h", CifSynthesisTool.codePrefix + "_library.c", CifSynthesisTool.codePrefix + "_library.h" };
		
		for (String generatedFile : files) {
            File file = new File(baseDirectory + "/controller/" + generatedFile);
            
            fileSystemAccess.generateFile(relativeOutputPath + generatedFile, Files.readString(file.toPath()));
		}
	}
}
