package nl.tue.robotsupervisorycontrollerdsl.generator.common;

import java.util.List;

public class GeneratorResult {
	private List<String> generatedFiles;
	
	public GeneratorResult(List<String> generatedFiles) {
		super();
		this.generatedFiles = generatedFiles;
	}

	public List<String> getGeneratedFiles() {
		return generatedFiles;
	}

	public void setGeneratedFiles(List<String> generatedFiles) {
		this.generatedFiles = generatedFiles;
	}
	
	
}
