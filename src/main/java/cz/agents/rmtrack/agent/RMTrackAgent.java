package cz.agents.rmtrack.agent;

import tt.euclid2d.Point;
import tt.euclid2d.Vector;
import tt.euclid2i.Trajectory;

public class RMTrackAgent extends Agent {
    private final Trajectory traj;
    private tt.euclid2i.Point position;

    public RMTrackAgent(int id, tt.euclid2d.Point start, tt.euclid2d.Point goal, float radius, float maxSpeed, Trajectory traj) {
        super(id, start, goal, radius, maxSpeed);
        this.position = start.toPoint2i();
        this.traj = traj;
    }

    @Override
    public Point getPosition() {
        return this.position.toPoint2d();
    }

    @Override
    public void tick(int timeMs, int deltaMs) {
        super.tick(timeMs, deltaMs);
        position = traj.get(timeMs);
    }

    public Trajectory getTrajectory() {
        return traj;
    }
}
