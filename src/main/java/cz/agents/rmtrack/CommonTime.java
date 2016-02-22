package cz.agents.rmtrack;

public class CommonTime {
	static long startedAt = System.currentTimeMillis(); 
	
	public static int currentTimeMs() {
		return (int) (System.currentTimeMillis() - startedAt);
	}
}
