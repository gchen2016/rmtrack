package cz.agents.rmtrack.agent;

import tt.euclid2d.Point;
import tt.euclid2d.Vector;
import tt.euclid2i.Trajectory;

public class RMTrackAgent extends Agent {
    private final Trajectory traj;

    public RMTrackAgent(int id, tt.euclid2d.Point start, tt.euclid2d.Point goal, float radius, float maxSpeed, Trajectory traj) {
        super(id, start, goal, radius, maxSpeed);

        this.traj = traj;
    }

    @Override
    public Vector getVelocity(int timeMs) {
        int LOOKAHEAD = 500;
        tt.euclid2i.Point carrot = traj.get(timeMs + LOOKAHEAD);

        Vector diff = new tt.euclid2d.Vector(carrot.toPoint2d());
        diff.sub(getPosition());
        if (diff.length() > 0.0001) {
            diff.normalize();
        }
        diff.scale(getMaxSpeed());

        return diff;
    }

    public Trajectory getTrajectory() {
        return traj;
    }
}
