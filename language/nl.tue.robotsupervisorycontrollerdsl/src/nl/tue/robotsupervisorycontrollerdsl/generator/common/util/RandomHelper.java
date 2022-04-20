package nl.tue.robotsupervisorycontrollerdsl.generator.common.util;

import java.util.Random;

public class RandomHelper {
	public static String generateString(int length) {
		String inputRange = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
        Random r = new Random();

        StringBuilder sb = new StringBuilder(length);

	    for (var i = 0; i < length; i++)
	        sb.append(inputRange.charAt(r.nextInt(inputRange.length())));
        
	    return sb.toString();
	}
}
