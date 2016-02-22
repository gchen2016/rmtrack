package cz.agents.rmtrack.agent;

import org.apache.log4j.Logger;
import tt.euclid2d.Point;


public class Agent {

    static final Logger LOGGER = Logger.getLogger(Agent.class);

    int id;
    Point start;
    Point goal;
    Point position;
    float radius;

    // Logging
	public long goalReachedSum;
	public long goalReachedSumSq;

    public Agent(int id, Point start, Point goal, float radius) {
        this.id = id;
        this.start = start;
        this.goal = goal;
        this.position = new Point(start);
        this.radius = radius;
    }

    public void tick(int timeMs) {
    	LOGGER.info(getName() + " Tick @ " + timeMs/1000.0 + "s");
    }

    public String getName() {
        return ""+id;
    }

    public boolean isDone() {
        final int EPS = 5;
        return position.distance(goal) < EPS;
    }

    public Point getPosition() {
        return position;
    }

    public float getRadius() {
        return radius;
    }
}
