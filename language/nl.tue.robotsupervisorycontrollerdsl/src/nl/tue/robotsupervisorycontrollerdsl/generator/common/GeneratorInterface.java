package nl.tue.robotsupervisorycontrollerdsl.generator.common;

import org.eclipse.xtext.generator.IFileSystemAccess2;

import nl.tue.robotsupervisorycontrollerdsl.robotSupervisoryControllerDSL.Robot;

public interface GeneratorInterface {
	GeneratorResult generate(Robot robot, IFileSystemAccess2 fileSystemAccess);
}
