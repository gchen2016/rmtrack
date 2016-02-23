package cz.agents.rmtrack.agent;

import org.apache.log4j.Logger;
import tt.euclid2d.Point;
import tt.euclid2d.Vector;


public abstract class Agent {

    static final Logger LOGGER = Logger.getLogger(Agent.class);
    int id;

    Point start;
    Point goal;
    float radius;
    private final float maxSpeed;

    boolean isDone = false;

    // Logging
	public long goalReachedSum = 0;
	public long goalReachedSumSq = 0;

    public Agent(int id, Point start, Point goal, float radius, float maxSpeed) {
        this.id = id;
        this.start = start;
        this.goal = goal;
        this.radius = radius;
        this.maxSpeed = maxSpeed;
    }

    public void tick(int timeMs, int deltaMs) {
    	//LOGGER.info(getName() + " Tick @ " + timeMs/1000.0 + "s");
        if (!isDone) {
            final int EPS = 10;
            if (getPosition().distance(goal) < EPS) {
                isDone = true;
                goalReachedSum = timeMs;
                goalReachedSumSq = (long) timeMs * (long) timeMs;
                LOGGER.info("finished at time: " + timeMs/1000);
            }
        }
    }

    public String getName() {
        return "" + id;
    }

    public boolean isDone() {
        return isDone;
    }

    abstract public Point getPosition();

    public float getRadius() {
        return radius;
    }

    public float getMaxSpeed() {
        return maxSpeed;
    }
}
