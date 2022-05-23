package nl.tue.robotsupervisorycontrollerdsl.generator.cpp.output;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.apache.commons.io.FileUtils;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.IFileSystemAccess2;

import com.google.inject.Singleton;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.FileHelper;
import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot;

@Singleton
public class OutputCopyUtil {
	public void copyDirectory(Robot robot, String inputPath, String outputPath, IFileSystemAccess2 fsa) throws IOException {
		Resource resource = robot.eResource();
		File originalFile = new File(FileHelper.findAbsolutePath(resource.getURI(), resource.getResourceSet()));
		
		File baseDirectory = originalFile.getParentFile();

		File source = new File(FileHelper.findAbsolutePath(inputPath, fsa, resource.getResourceSet()));
		File target = constructFile(Paths.get(outputPath), baseDirectory.toPath());

		FileUtils.copyDirectory(source, target);
	}
	
	public File constructFile(Path path, Path base) {
		if (path.isAbsolute()) {
			return path.toFile();
		} else {
			return base.resolve(path).toFile();
		}
	}
}
