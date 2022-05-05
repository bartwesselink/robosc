package nl.tue.robotsupervisorycontrollerdsl.generator.common.util;

import java.util.Random;

public class RandomHelper {
	public static String generateString(int length) {
		return generateString(length, new Random());
	}
	
	public static String generateString(int length, Random random) {
		String inputRange = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

        StringBuilder sb = new StringBuilder(length);

	    for (var i = 0; i < length; i++)
	        sb.append(inputRange.charAt(random.nextInt(inputRange.length())));
        
	    return sb.toString();
	}
}
