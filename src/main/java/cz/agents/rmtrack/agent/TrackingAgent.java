package cz.agents.rmtrack.agent;

import cz.agents.rmtrack.util.Disturbance;
import tt.euclid2d.Point;
import tt.euclid2i.Trajectory;

import java.util.List;

public class TrackingAgent extends Agent {
    public enum TrackingMethod {ALLSTOP, RMTRACK};

    private final Trajectory traj;
    private TrackingMethod trackingMethod;

    private int planPos = 0;
    private boolean currentlyDisturbed = false;
    private boolean currentlyWaiting = false;

    public TrackingAgent(int id, tt.euclid2d.Point start, tt.euclid2d.Point goal, float radius, float maxSpeed,
                         Trajectory traj, Disturbance disturbance, TrackingMethod trackingMethod, int baseTaskDuration) {
        super(id, start, goal, radius, maxSpeed, disturbance, baseTaskDuration);
        this.traj = traj;
        this.trackingMethod = trackingMethod;
    }

    @Override
    public Point getPosition() {
        return this.traj.get(planPos).toPoint2d();
    }

    @Override
    public void update(int t_current_ms, int t_next_ms, List<Agent> agents) {
        if (!isAtGoal()) {
            travelTime = t_current_ms;
        }

        int deltaT = t_next_ms - t_current_ms;

        boolean proceed = true;

        if (trackingMethod == TrackingMethod.ALLSTOP) {
            proceed = !someoneDisturbed(agents, t_current_ms);
        }

        else if (trackingMethod == TrackingMethod.RMTRACK) {
            proceed = true;
            for (int j = 0; j < agents.size(); j++) {
                if (j != id) {
                    TrackingAgent otherAgent = (TrackingAgent) agents.get(j);
                    if (this.getPlanPos() > otherAgent.getPlanPos()) {
                        if (spatialCollision(this.getTrajectory(), this.getPlanPos(), this.getPlanPos() + deltaT,
                                otherAgent.getTrajectory(), otherAgent.getPlanPos(), this.getPlanPos() + deltaT,
                                deltaT / 2, (int) (this.getRadius() + otherAgent.getRadius()))) {
                            proceed = false;
                        }
                    }
                }
            }
        }

        currentlyWaiting = !proceed;

        if (isDisturbed(t_current_ms)) {
            // disturbed, no progress
            currentlyDisturbed = true;
        } else  {
            // disturbed, progress
            currentlyDisturbed = false;
            if (proceed) {
                planPos += deltaT;
            }
        }
    }

    private boolean someoneDisturbed(List<Agent> agents, int timeMs) {
        for (Agent agent: agents) {
            if (agent.isDisturbed(timeMs)) {
                return true;
            }
        }
        return false;
    }


    private boolean spatialCollision(Trajectory traj1, int start1 , int end1, Trajectory traj2, int start2, int end2, int sampling, int separation) {
        for (int t1 = start1; t1 < end1; t1 += sampling) {
            for (int t2 = start2; t2 < end2; t2 += sampling ) {
                if (traj1.get(t1).distance(traj2.get(t2)) < separation) {
                    return true;
                }
            }
        }

        return false;
    }

    public Trajectory getTrajectory() {
        return traj;
    }

    @Override
    public boolean wasDisturbed() {
        return currentlyDisturbed;
    }

    public int getPlanPos() {
        return planPos;
    }

    public boolean isCurrentlyWaiting() {
        return currentlyWaiting;
    }
}
