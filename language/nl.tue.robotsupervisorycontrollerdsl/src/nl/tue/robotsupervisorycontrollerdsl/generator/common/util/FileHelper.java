package nl.tue.robotsupervisorycontrollerdsl.generator.common.util;

import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.Path;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.generator.IFileSystemAccess2;

public class FileHelper {
	public static String findAbsolutePath(String fileName, IFileSystemAccess2 fsa, ResourceSet resourceSet) {
		URI relative = fsa.getURI(fileName);

		return findAbsolutePath(relative, resourceSet);
	}
	
	public static String findAbsolutePath(URI base, ResourceSet resourceSet) {
		String result;

		if (base.isPlatform()) {
			Path path = new Path(base.toPlatformString(true));
			
            result = ResourcesPlugin.getWorkspace().getRoot().getFile(path).getRawLocation().toOSString();
        } else {
        	result = resourceSet.getURIConverter().normalize(base).toFileString();
        }
		
		result = result.replaceAll("^///", "/");
		
		return result;
	}
}
