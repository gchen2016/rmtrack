package cz.agents.rmtrack;

import cz.agents.rmtrack.util.BestResponse;
import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.AStarShortestPathSimple;
import org.jgrapht.util.HeuristicToGoal;
import org.jgrapht.util.heuristics.ZeroHeuristic;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.region.Circle;
import tt.euclidtime3i.region.MovingCircle;
import tt.jointeuclid2ni.probleminstance.EarliestArrivalProblem;

import java.io.IOException;
import java.util.Collection;
import java.util.LinkedList;

public class RPP {
    public static int RADIUS_GRACE = 0;
    static Logger LOGGER = Logger.getLogger(RPP.class);

    public static EvaluatedTrajectory[] findCollisionFreeTrajs(EarliestArrivalProblem problem, int timeStep, int maxTime) {
        return solve(problem, timeStep, maxTime, false);
    }

    public static EvaluatedTrajectory[] findCollidingTrajs(EarliestArrivalProblem problem, int timeStep, int maxTime) {
        return solve(problem, timeStep, maxTime, true);
    }

    public static EvaluatedTrajectory[] solve(EarliestArrivalProblem problem, int timeStep, int maxTime, boolean allowCollisions) {
        final EvaluatedTrajectory[] trajs = new EvaluatedTrajectory[problem.nAgents()];

        long programStartedAtNs = System.nanoTime();

        for (int i = 0; i < problem.nAgents(); i++) {
            Collection<Region> sObst = new LinkedList<Region>();
            Collection<tt.euclidtime3i.Region> dObst = new LinkedList<tt.euclidtime3i.Region>();

            if (!allowCollisions) {
                for (int j = 0; j < problem.nAgents(); j++) {
                    if (j < i) {
                        int samplingInterval = timeStep / 2;
                        dObst.add(new MovingCircle(trajs[j], problem.getBodyRadius(i) + problem.getBodyRadius(j), samplingInterval));
                    } else if (j > i) {
                        sObst.add(new Circle(problem.getStart(j), problem.getBodyRadius(i) + problem.getBodyRadius(j)));
                    }
                }
            }

            LOGGER.info("Computing trajectory for robot " + i + " from " + problem.getStart(i) + " to " + problem.getTarget(i));
            EvaluatedTrajectory traj = BestResponse.computeBestResponse(
                    problem.getStart(i),
                    problem.getTarget(i),
                    problem.getMaxSpeed(i),
                    problem.getPlanningGraph(),
                    sObst,
                    dObst,
                    maxTime,
                    timeStep);

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


    public static double[] findBaseTaskDurations(EarliestArrivalProblem problem) {
        double[] duration = new double[problem.nAgents()];
        for (int i=0; i<problem.nAgents(); i++) {
            DirectedGraph<Point, Line> graph = problem.getPlanningGraph();
            GraphPath<Point, Line> path = AStarShortestPathSimple.findPathBetween(graph, new ZeroHeuristic<Point>(), problem.getStart(i), problem.getTarget(i));
            duration[i] = path.getWeight() / problem.getMaxSpeed(i);
        }

        return duration;
    }

}
