package cz.agents.rmtrack.agent;

import cz.agents.rmtrack.util.Disturbance;
import tt.euclid2d.Point;
import tt.euclid2d.Vector;
import tt.euclid2i.Trajectory;

import java.util.List;

public class RMTrackAgent extends Agent {
    private final Trajectory traj;

    private int planPos = 0;
    private boolean currentlyDisturbed = false;

    public RMTrackAgent(int id, tt.euclid2d.Point start, tt.euclid2d.Point goal, float radius, float maxSpeed,
                        Trajectory traj, Disturbance disturbance) {
        super(id, start, goal, radius, maxSpeed, disturbance);
        this.traj = traj;
    }

    @Override
    public Point getPosition() {
        return this.traj.get(planPos).toPoint2d();
    }

    @Override
    public void update(int t_current_ms, int t_next_ms, List<Agent> agents) {
        if (isDisturbed(t_current_ms)) {
            currentlyDisturbed = true;
            // disturbed
        } else {
            currentlyDisturbed = false;
            int deltaT = t_next_ms - t_current_ms;
            planPos += deltaT;
        }
    }

    public Trajectory getTrajectory() {
        return traj;
    }

    @Override
    public boolean isCurrentlyDisturbed() {
        return currentlyDisturbed;
    }
}
