package cz.agents.rmtrack;

import cz.agents.rmtrack.util.BestResponse;
import org.apache.log4j.Logger;
import org.jgrapht.util.HeuristicToGoal;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Region;
import tt.euclid2i.region.Circle;
import tt.euclidtime3i.Point;
import tt.euclidtime3i.ShortestPathHeuristic;
import tt.euclidtime3i.region.MovingCircle;
import tt.jointeuclid2ni.probleminstance.EarliestArrivalProblem;

import java.io.IOException;
import java.util.Collection;
import java.util.LinkedList;

public class RPP {
    public static int RADIUS_GRACE = 0;
    static Logger LOGGER = Logger.getLogger(RPP.class);

    public static EvaluatedTrajectory[] solve(EarliestArrivalProblem problem, int timeStep, int maxTime) {
        final EvaluatedTrajectory[] trajs = new EvaluatedTrajectory[problem.nAgents()];

        long programStartedAtNs = System.nanoTime();

        for (int i = 0; i < problem.nAgents(); i++) {
            Collection<Region> sObst = new LinkedList<Region>();
            Collection<tt.euclidtime3i.Region> dObst = new LinkedList<tt.euclidtime3i.Region>();

            for (int j = 0; j < problem.nAgents(); j++) {
                if (j < i) {
                    int samplingInterval = timeStep/2;
                    dObst.add(new MovingCircle(trajs[j], problem.getBodyRadius(i) + problem.getBodyRadius(j), samplingInterval));
                } else if (j > i) {
                    sObst.add(new Circle(problem.getStart(j), problem.getBodyRadius(i) + problem.getBodyRadius(j)));
                }
            }

            LOGGER.info("Computing trajectory for robot " + i);
            EvaluatedTrajectory traj = BestResponse.computeBestResponse(
                    problem.getStart(i),
                    problem.getTarget(i),
                    problem.getMaxSpeed(i),
                    problem.getPlanningGraph(),
                    sObst,
                    dObst,
                    maxTime,
                    timeStep);

            try {
                System.in.read();
            } catch (IOException e) {
                e.printStackTrace();
            }

            if (traj == null) {
                LOGGER.error("===== PATH PLANNING FOR ROBOT " + i + "FAILED =====");
                break;
            } else {
                trajs[i] = traj;
            }

        }

        LOGGER.info("Path planning finished...");
        return trajs;
    }
}
