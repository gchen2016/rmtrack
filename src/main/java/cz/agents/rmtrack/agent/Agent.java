package cz.agents.rmtrack.agent;

import cz.agents.rmtrack.util.Disturbance;
import org.apache.log4j.Logger;
import tt.euclid2d.Point;

import java.util.List;


public abstract class Agent {

    static final Logger LOGGER = Logger.getLogger(Agent.class);
    int id;

    Point start;
    Point goal;
    float radius;
    final float maxSpeed;
    final Disturbance disturbance;

    // Logging
    public int travelTime;

    public Agent(int id, Point start, Point goal, float radius, float maxSpeed, Disturbance disturbance) {
        this.id = id;
        this.start = start;
        this.goal = goal;
        this.radius = radius;
        this.maxSpeed = maxSpeed;
        this.disturbance = disturbance;
        this.travelTime = 0;
    }

    abstract public void update(int t_current_ms, int t_next_ms, List<Agent> agents);

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

    public abstract boolean wasDisturbed();

    boolean isDisturbed(int t){
        return disturbance.isDisturbed(id, t);
    }

    public Point getGoal() {
        return goal;
    }

    public boolean isAtGoal() {
        double EPS = 0.9;
        return getPosition().epsilonEquals(getGoal(), EPS);
    }
}
