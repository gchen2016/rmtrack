package cz.agents.rmtrack;
import cz.agents.alite.simulation.vis.SimulationControlLayer;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.terminal.textBackgroundLayer.TextBackgroundLayer;
import cz.agents.alite.vis.layer.toggle.KeyToggleLayer;
import cz.agents.rmtrack.agent.Agent;
import cz.agents.rmtrack.agent.ORCAAgent;
import cz.agents.rmtrack.agent.TrackingAgent;
import cz.agents.rmtrack.util.Disturbance;
import org.apache.log4j.Logger;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Trajectory;
import tt.jointeuclid2ni.probleminstance.EarliestArrivalProblem;
import tt.jointeuclid2ni.probleminstance.TrajectoryCoordinationProblemXMLDeserializer;
import tt.jointeuclid2ni.probleminstance.VisUtil;
import tt.jointeuclid2ni.solver.Parameters;
import tt.util.AgentColors;
import tt.util.Args;
import tt.vis.FastTrajectoriesLayer;
import tt.vis.FastTrajectoriesLayer.ColorProvider;
import tt.vis.FastTrajectoriesLayer.TrajectoriesProvider;
import tt.vis.LabeledCircleLayer;

import java.awt.*;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;


public class ScenarioCreator {
	
	/* 
	 * Units:
	 * time: 1 time unit = 1ms; 
	 * distance: 1 distance unit = depending on the map, 2cm typically. 
	 * speed: in du/ms (distance units / millisecond), typically 0.05 du/ms represents roughly 1m/1s 
	 */

	////////////////////////////////////////////////////////////////////////

    public static void main(String[] args) {
    	createFromArgs(args);
    }

    ///////////////////////////////////////////////////////////////////////
    
    static Logger LOGGER = Logger.getLogger(ScenarioCreator.class);

    enum Method {
        ALLSTOP,
        RMTRACK,
        ORCA_T,
        ORCA
    }


    private static EarliestArrivalProblem problem;

    public static void createFromArgs(String[] args) {
    	Parameters params = new Parameters();
    	 	
    	String xml = Args.getArgumentValue(args, "-problemfile", true);
    	String methodStr = Args.getArgumentValue(args, "-method", true);
    	String maxTimeStr = Args.getArgumentValue(args, "-maxtime", true);
    	params.maxTime = Integer.parseInt(maxTimeStr);
    	String timeStepStr = Args.getArgumentValue(args, "-timestep", true);
    	params.timeStep = Integer.parseInt(timeStepStr);
    	params.showVis = Args.isArgumentSet(args, "-showvis");
    	params.verbose = Args.isArgumentSet(args, "-verbose");
    	String timeoutStr = Args.getArgumentValue(args, "-timeout", false);
        params.summaryPrefix = Args.getArgumentValue(args, "-summaryprefix", false, "");
        params.activityLogFile = Args.getArgumentValue(args, "-activitylog", false, null);
        String bgImgFileName = Args.getArgumentValue(args, "-bgimg", false, null);
        String simSpeedStr = Args.getArgumentValue(args, "-simspeed", false, "1");
        params.simSpeed = Double.parseDouble(simSpeedStr);

        String disturbanceProbStr = Args.getArgumentValue(args, "-dprob", false, "10");

        String[] parts = disturbanceProbStr.split("s", -1);
        params.disturbanceProbs = new double[parts.length];
        for (int i = 0; i < parts.length; i++) {
            params.disturbanceProbs[i] = Integer.parseInt(parts[i]) / 100.0;
        }

        String disturbanceSeedStr = Args.getArgumentValue(args, "-dseed", false, "1");
        params.disturbanceSeed = Integer.parseInt(disturbanceSeedStr);

        String disturbanceQuantStr = Args.getArgumentValue(args, "-dquant", false, "1000");
        params.disturbanceQuantum = Integer.parseInt(disturbanceQuantStr);

		File file = new File(xml);
	    params.fileName = file.getName();
	    
	    // Load the PNG image as a background, if provided
	    if (bgImgFileName != null) {
		    File bgImgFile = new File(bgImgFileName);
		    if (bgImgFile.exists()) {
		    	params.bgImageFile = bgImgFile;
		    }        	
        }

	    try {
			problem = TrajectoryCoordinationProblemXMLDeserializer.deserialize(new FileInputStream(file));
		} catch (FileNotFoundException e) {
			throw new RuntimeException(e);
		}

	    Method method = Method.valueOf(methodStr);
	    
	    params.runtimeDeadlineMs = 3600*1000; /* default timeout is 1 hour */
	    if (timeoutStr != null) {
	    	int timeout = Integer.parseInt(timeoutStr);
	    	params.runtimeDeadlineMs = timeout;
	    	killAt(System.currentTimeMillis() + timeout, params.summaryPrefix, params.noOfClusters);
	    }

        switch (method) {
            case ALLSTOP:
            case RMTRACK:
            case ORCA_T:
                solveTracking(problem, method, params);
                break;


            case ORCA:
                solveORCA(problem, params);
                break;

            default:
                throw new RuntimeException("Unknown method");

        }
    }

    private static void killAt(final long killAtMs, final String summaryPrefix, final int clusters) {
    	Thread t = new Thread() {
			@Override
			public void run() {
				while (System.currentTimeMillis() < killAtMs) {
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {}
				}
				printSummary(summaryPrefix, Status.TIMEOUT, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1);
				System.exit(0);
			}
    	};
    	t.start();
	}

	private static void solveTracking(final EarliestArrivalProblem problem,
                                      Method method, final Parameters params) {

        // find minimum lengths
        int[] durationsOverShortestPath = Util.computeDurationOverShortestPath(problem);

        // find trajectories
        final EvaluatedTrajectory[] trajs = Util.findCollisionFreeTrajs(problem, params.timeStep, params.maxTime);

        for (double disturbanceProb : params.disturbanceProbs) {

            if (params.showVis) {
                VisUtil.initVisualization(problem.getEnvironment(), "RMTRACK", params.bgImageFile, params.timeStep/2);
                VisUtil.visualizeEarliestArrivalProblem(problem);
                visualizeTrajectories(trajs, params.timeStep);
            }

            Disturbance disturbance = new Disturbance((float) disturbanceProb, params.disturbanceQuantum, params.disturbanceSeed, problem.nAgents());
            final int[] lowerBoundDurations = Util.findLowerBoundDuration(problem, trajs, disturbance);
            final int[] d0Durations = Util.findLowerBoundDuration(problem, trajs, new Disturbance(0, 1000, 1, problem.nAgents()));

            List<Agent> agents = new LinkedList<>();
            for (int i=0; i < problem.nAgents(); i++) {
                Agent agent = null;

                switch (method) {
                    case ALLSTOP:
                        agent = new TrackingAgent(i,
                                problem.getStart(i).toPoint2d(),
                                problem.getTarget(i).toPoint2d(),
                                problem.getBodyRadius(i),
                                problem.getMaxSpeed(i),
                                trajs[i],
                                disturbance,
                                TrackingAgent.TrackingMethod.ALLSTOP);
                        break;

                    case RMTRACK:
                        agent = new TrackingAgent(i,
                                problem.getStart(i).toPoint2d(),
                                problem.getTarget(i).toPoint2d(),
                                problem.getBodyRadius(i),
                                problem.getMaxSpeed(i),
                                trajs[i],
                                disturbance,
                                TrackingAgent.TrackingMethod.RMTRACK);
                        break;

                    case ORCA_T:
                        agent = new ORCAAgent(i,
                                problem.getStart(i).toPoint2d(),
                                problem.getTarget(i).toPoint2d(),
                                problem.getEnvironment(),
                                trajs[i],
                                problem.getBodyRadius(i),
                                problem.getMaxSpeed(i),
                                disturbance,
                                new Random(params.disturbanceSeed),
                                params.showVis);
                }

                agents.add(i, agent);
            }

            // simulate execution
            simulate(problem, agents, params);
            computeStatistics(agents, disturbanceProb,  durationsOverShortestPath, d0Durations, lowerBoundDurations, params);
            VisManager.unregisterLayers();
        }

        //System.exit(0);
    } 
	
	private static void solveORCA(final EarliestArrivalProblem problem, final Parameters params) {
        for (double disturbanceProb : params.disturbanceProbs) {

            int[] durationsOverShortestPath = Util.computeDurationOverShortestPath(problem);

            if (params.showVis) {
                VisUtil.initVisualization(problem.getEnvironment(), "RMTRACK", params.bgImageFile, params.timeStep/2);
                VisUtil.visualizeEarliestArrivalProblem(problem);
            }

            Disturbance disturbance = new Disturbance((float) disturbanceProb, params.disturbanceQuantum, params.disturbanceSeed, problem.nAgents());

            List<Agent> agents = new LinkedList<>();
            for (int i=0; i < problem.nAgents(); i++) {
                agents.add(i,
                        new ORCAAgent(i,
                                problem.getStart(i).toPoint2d(),
                                problem.getTarget(i).toPoint2d(),
                                problem.getEnvironment(),
                                problem.getPlanningGraph(),
                                problem.getBodyRadius(i),
                                problem.getMaxSpeed(i),
                                disturbance,
                                new Random(params.disturbanceSeed),
                                false/*params.showVis*/ ));
            }

            // simulate execution
            simulate(problem, agents, params);

            int[] zeros = new int[problem.nAgents()];

            computeStatistics(agents, disturbanceProb, durationsOverShortestPath, zeros, zeros, params);
            VisManager.unregisterLayers();
        }

        //System.exit(0);
    }

    private static void simulate(final EarliestArrivalProblem problem, List<Agent> agents, final Parameters params) {
    	
        initAgentVisualization(agents, params.timeStep);

        int SIMULATION_STEP_MS = 100;
        int SIMULATE_UNTIL = 600000;

        class SimulationControl {
            public int simulatedTimeMs = 0;
            public float simSpeed = 1;
            public boolean running = true;
        }

        final SimulationControl simControl = new SimulationControl();

        // Simulation Control Layer
        VisManager.registerLayer(SimulationControlLayer.create(new SimulationControlLayer.SimulationControlProvider() {

            @Override
            public void setSpeed(float f) {simControl.simSpeed = f;}

            @Override
            public void setRunning(boolean running) {simControl.running = running;}

            @Override
            public boolean isRunning() { return simControl.running; }

            @Override
            public double getTime() { return (simControl.simulatedTimeMs / 1000.0); }

            @Override
            public float getSpeed() { return simControl.simSpeed; }
        }));

        try {
            System.in.read();
        } catch (IOException e) {
            e.printStackTrace();
        }

        while (!allDone(agents) && simControl.simulatedTimeMs < SIMULATE_UNTIL){
            if (simControl.running == true) {

                for (Agent agent : agents) {
                    agent.update(simControl.simulatedTimeMs, simControl.simulatedTimeMs + SIMULATION_STEP_MS, agents);
                }

                if (params.showVis) {
                    try {
                        Thread.sleep((int) ((double) SIMULATION_STEP_MS * (double) simControl.simSpeed));
                    } catch (InterruptedException e) {
                    }
                }

                simControl.simulatedTimeMs += SIMULATION_STEP_MS;
            } else {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) { }
            }
        }
    }

    private static void computeStatistics(List<Agent> agents, double disturbance, int[] shortestPathDurations, int[] d0Durations, int[] lbDurations, Parameters params) {
        long spSum = 0;
        long d0Sum = 0;
        long lbSum = 0;

        long travelTimeSum = 0;
        long travelTimeSumSq = 0;

        long prolongSpSum = 0;
        long prolongSpSumSq = 0;

        long prolongD0Sum = 0;
        long prolongD0SumSq = 0;

        long prolongLBSum = 0;
        long prolongLBSumSq = 0;

        long makespanAbs = 0;
        long makespanSPProlong = 0;
        long makespanD0Prolong = 0;
        long makespanLBProlong = 0;


        for (int i=0; i<agents.size(); i++) {
            Agent agent = agents.get(i);
            spSum += shortestPathDurations[i];
            d0Sum += d0Durations[i];
            lbSum += lbDurations[i];

            LOGGER.debug("Agent " + i + " finished at " + agent.travelTime);

            travelTimeSum += agent.travelTime;
            travelTimeSumSq += travelTimeSum * travelTimeSum;

            long prolongSp = agent.travelTime - shortestPathDurations[i];
            long prolongD0 = agent.travelTime - d0Durations[i];
            long prolongLB = agent.travelTime - lbDurations[i];

            prolongSpSum += prolongSp;
            prolongD0Sum += prolongD0;
            prolongLBSum += prolongLB;

            prolongSpSumSq += prolongSp * prolongSp;
            prolongD0SumSq += prolongD0 * prolongD0;
            prolongLBSumSq += prolongLB * prolongLB;

            if (agent.travelTime > makespanAbs)
                makespanAbs = agent.travelTime;

            if ((agent.travelTime - shortestPathDurations[i]) > makespanSPProlong)
                makespanSPProlong = (agent.travelTime - shortestPathDurations[i]);

            if ((agent.travelTime - d0Durations[i]) > makespanD0Prolong)
                makespanD0Prolong = (agent.travelTime - d0Durations[i]);

            if ((agent.travelTime - lbDurations[i]) > makespanLBProlong)
                makespanLBProlong = (agent.travelTime - lbDurations[i]);
        }

        long n = agents.size();

        Status status;
        if (allDone(agents)) {
            status = Status.SUCCESS;
        } else {
            status = Status.FAIL;
        }

        // status;dprob;dquant;dseed;spSum;d0Sum;lbSum;travelTimeSum;travelTimeSumSq;prolongSpSum;prolongSpSumSq;prolongD0Sum;prolongD0SumSq;prolongLBSum;prolongLBSumSq;makespanAbs;makespanSPProlong;makespanD0Prolong;makespanLBProlong

        printSummary(params.summaryPrefix, status, disturbance, params.disturbanceQuantum, params.disturbanceSeed,
                spSum,d0Sum,lbSum,travelTimeSum,travelTimeSumSq,prolongSpSum,prolongSpSumSq,
                prolongD0Sum,prolongD0SumSq,prolongLBSum,prolongLBSumSq,makespanAbs,makespanSPProlong,makespanD0Prolong,makespanLBProlong);
    }
    
    private static boolean allDone(List<Agent> agents) {
    	for (final Agent agent : agents) {
            agent.isAtGoal();
    		if (!agent.isAtGoal()) {
                return false;
            }
    	}
    	return true;
	}


    private static void visualizeTrajectories(Trajectory[] trajs, int timestep) {
        // trajectories
        VisManager.registerLayer(
                KeyToggleLayer.create("t", true,
                        FastTrajectoriesLayer.create(new TrajectoriesProvider() {

                            @Override
                            public Trajectory[] getTrajectories() {
                                return trajs;
                            }
                        },new ColorProvider() {

                            @Override
                            public Color getColor(int i) {
                                return AgentColors.getColorForAgent(i);
                            }
                        }, 3, timestep)));
    }

	private static void initAgentVisualization(final List<Agent> agents, int timeStep) {
        // goals
        VisManager.registerLayer(
                KeyToggleLayer.create("g", true,
                        LabeledCircleLayer.create(new LabeledCircleLayer.LabeledCircleProvider<tt.euclid2d.Point>() {

                            @Override
                            public Collection<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>> getLabeledCircles() {
                                LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>> list = new LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>>();
                                for (int i = 0; i < agents.size(); i++) {
                                    tt.euclid2d.Point pos = agents.get(i).getGoal();

                                    list.add(new LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>(pos,
                                            (int) agents.get(i).getRadius(), ""  , AgentColors.getColorForAgent(i).brighter(),
                                            null,
                                            null));
                                }

                                return list;
                            }

                        }, new tt.euclid2d.vis.ProjectionTo2d())));

        // positions
        VisManager.registerLayer(
        	KeyToggleLayer.create("b", true, 
        	LabeledCircleLayer.create(new LabeledCircleLayer.LabeledCircleProvider<tt.euclid2d.Point>() {

             @Override
             public Collection<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>> getLabeledCircles() {
                 LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>> list = new LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>>();
                 for (int i = 0; i < agents.size(); i++) {
                     tt.euclid2d.Point pos = agents.get(i).getPosition();



                     Color textColor = Color.WHITE;
                     Color fillColor = AgentColors.getColorForAgent(i);

                     if (agents.get(i) instanceof TrackingAgent) {
                         TrackingAgent agent = (TrackingAgent) agents.get(i);
                         if (agent.isCurrentlyWaiting()) {
                             fillColor = Color.LIGHT_GRAY;
                         }
                     }

                     if (agents.get(i).wasDisturbed()) {
                         fillColor = Color.black;
                     }

                     String text = ""; //+ i;

                     if (agents.get(i).isAtGoal()) {
                         text = String.format("%.0fs", (double) agents.get(i).travelTime / 1000.0);
                     }

                	 list.add(new LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>(pos,
                             (int) agents.get(i).getRadius(),
                             text,
                             AgentColors.getColorForAgent(i),
                             fillColor ,
                             textColor));
                 }

                 return list;
             }

         }, new tt.euclid2d.vis.ProjectionTo2d())));

        // planned positions
        VisManager.registerLayer(
                KeyToggleLayer.create("x", false,
                        LabeledCircleLayer.create(new LabeledCircleLayer.LabeledCircleProvider<tt.euclid2d.Point>() {

                            @Override
                            public Collection<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>> getLabeledCircles() {
                                LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>> list = new LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>>();
                                for (int i = 0; i < agents.size(); i++) {
                                    tt.euclid2d.Point pos = agents.get(i).getPlannedPosition();

                                    list.add(new LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>(pos,
                                            (int) agents.get(i).getRadius(), ""  , AgentColors.getColorForAgent(i),
                                            null ,
                                            Color.DARK_GRAY));
                                }

                                return list;
                            }

                        }, new tt.euclid2d.vis.ProjectionTo2d())));

 	}

    enum Status {SUCCESS, FAIL, TIMEOUT}

	static long avg(long sum, long n) {
		return sum / n;
	}

    private static void printSummary(String prefix, Status status, double dprob, int dquant, int dseed,
                                     long spSum, long d0Sum, long lbSum, long travelTimeSum, long travelTimeSumSq, long prolongSpSum, long prolongSpSumSq, long
                                     prolongD0Sum, long prolongD0SumSq, long prolongLBSum, long prolongLBSumSq, long makespanAbs, long makespanSPProlong,
                                     long makespanD0Prolong, long makespanLBProlong) {
        System.out.println(prefix +
                status.toString() + ";" +
                dprob + ";" +
                dquant + ";" +
                dseed + ";" +
                spSum + ";" +
                d0Sum + ";" +
                lbSum + ";" +
                travelTimeSum + ";" +
                travelTimeSumSq + ";" +
                prolongSpSum + ";" +
                prolongSpSumSq + ";" +
                prolongD0Sum + ";" +
                prolongD0SumSq + ";" +
                prolongLBSum + ";" +
                prolongLBSumSq + ";" +
                makespanAbs + ";" +
                makespanSPProlong + ";" +
                makespanD0Prolong + ";" +
                makespanLBProlong + ";");

    }

	static long sd(long sumSq, long mean, long n) {
		double var = (sumSq/n) - (mean*mean); 
		return (long) Math.round(Math.sqrt(var));
	}

}
