package nl.tue.robotsupervisorycontrollerdsl.generator.config;

import java.io.File;
import java.io.IOException;
import java.io.Reader;
import java.nio.file.Files;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.IFileSystemAccess2;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import com.google.inject.Singleton;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.FileHelper;
import nl.tue.robotsupervisorycontrollerdsl.generator.config.exception.ConfigParseException;
import nl.tue.robotsupervisorycontrollerdsl.generator.config.model.Config;

@Singleton
public class ConfigService {
	public static final String CONFIG_NAME = "controller-config.json";
	
	public Config getConfig(Resource resource, IFileSystemAccess2 fsa) {
		URI location = resource.getURI();
		
		String fileName = FileHelper.findAbsolutePath(location, resource.getResourceSet());

		File directory = new File(fileName).getParentFile();
		File config = findConfig(directory);

		if (config != null) {
			Gson gson = new Gson();
			
			try {
				Reader reader = Files.newBufferedReader(config.toPath());
				Config result = gson.fromJson(reader, new TypeToken<Config>(){}.getType());

				return result;
			} catch (IOException e) {
				throw new ConfigParseException();
			}

		}

		return new Config();
	}
	
	private static File findConfig(File source) {
	    if (source.isDirectory()) {
	        File[] files = source.listFiles();
	        for (File file : files) {
	            File found = findConfig(file);

	            if (found != null) {
	                return found;
	            }
	        }
	    } else {
	        if (CONFIG_NAME.equals(source.getName())) {
	            return source;
	        }
	    }

	    return null;
	}
}
