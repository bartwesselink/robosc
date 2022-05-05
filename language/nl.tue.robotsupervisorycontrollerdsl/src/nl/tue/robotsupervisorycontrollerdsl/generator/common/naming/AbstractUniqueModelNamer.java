package nl.tue.robotsupervisorycontrollerdsl.generator.common.naming;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import javax.inject.Singleton;

import org.eclipse.emf.ecore.EObject;

import nl.tue.robotsupervisorycontrollerdsl.generator.common.util.RandomHelper;

@Singleton
abstract class AbstractUniqueModelNamer<T extends EObject> {
	private Map<T, String> cache = new HashMap<>();
	protected String prefix = "p";
	protected int length = 12;

	/**
	 * Returns the unique name of an entity or generates one if it was not yet
	 * determined. Uses a default random seed.
	 * 
	 * @param entity
	 * @return the name of the entity
	 */
	public String getName(T entity) {
		return getName(entity, new Random());
	}

	/**
	 * Returns the unique name of an entity or generates one if it was not yet
	 * determined
	 * 
	 * @param entity
	 * @param random
	 * @return the name of the entity
	 */
	public String getName(T entity, Random random) {
		if (!cache.containsKey(entity)) {
			cache.put(entity, generateName(random));
		}

		return cache.get(entity);
	}

	/**
	 * Generate a non-existing name
	 * 
	 * @param random
	 * @return the unique name
	 */
	protected String generateName(Random random) {
		String current = null;

		while (current == null || cache.containsValue(current)) {
			current = prefix + RandomHelper.generateString(12, random);
		}
		
		return current;
	}
}
