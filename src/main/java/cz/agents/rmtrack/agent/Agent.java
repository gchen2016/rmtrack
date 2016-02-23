package cz.agents.rmtrack.agent;

import cz.agents.rmtrack.util.Disturbance;
import org.apache.log4j.Logger;
import tt.euclid2d.Point;
import tt.euclid2d.Vector;

import java.util.List;


public abstract class Agent {

    static final Logger LOGGER = Logger.getLogger(Agent.class);

    int id;
    Point start;
    Point goal;
    float radius;
    final float maxSpeed;
    final Disturbance disturbance;


    boolean isDone = false;

    // Logging
	public long goalReachedSum = 0;
	public long goalReachedSumSq = 0;

    public Agent(int id, Point start, Point goal, float radius, float maxSpeed, Disturbance disturbance) {
        this.id = id;
        this.start = start;
        this.goal = goal;
        this.radius = radius;
        this.maxSpeed = maxSpeed;
        this.disturbance = disturbance;
    }

    abstract public void update(int t_current_ms, int t_next_ms, List<Agent> agents);

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

    public Disturbance getDisturbance() {
        return disturbance;
    }

    public abstract boolean isCurrentlyDisturbed();

    boolean isDisturbed(int t){
        return disturbance.isDisturbed(id, t);
    }
}
