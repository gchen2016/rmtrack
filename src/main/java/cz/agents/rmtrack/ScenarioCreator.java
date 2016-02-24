package cz.agents.rmtrack;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.toggle.KeyToggleLayer;
import cz.agents.rmtrack.agent.Agent;
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
    
    static long simulationStartedAt;
    static Logger LOGGER = Logger.getLogger(ScenarioCreator.class);

    enum Method {
        ALLSTOP,
        RMTRACK,
        ORCA}


    private static EarliestArrivalProblem problem;

    public static void createFromArgs(String[] args) {
    	simulationStartedAt = System.currentTimeMillis();
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

        String disturbanceProbStr = Args.getArgumentValue(args, "-dprob", false, "0.1");
        params.disturbanceProb = Double.parseDouble(disturbanceProbStr);

        String disturbanceSeedStr = Args.getArgumentValue(args, "-dseed", false, "1");
        params.disturbanceSeed = Integer.parseInt(disturbanceSeedStr);

        String disturbanceQuantStr = Args.getArgumentValue(args, "-dquant", false, "1000");
        params.disturbanceQuantum = Integer.parseInt(disturbanceQuantStr);

        String seedStr = Args.getArgumentValue(args, "-seed", true);
    	params.random = new Random(Integer.parseInt(seedStr));
    	
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
	    
    	create(problem, method, params);
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
				printSummary(summaryPrefix, Status.TIMEOUT, -1, -1, -1, -1, -1, -1, -1, -1, -1);
				System.exit(0);
			}
    	};
    	t.start();
	}


	public static void create(EarliestArrivalProblem problem, Method method, final Parameters params) {
		
        if (params.showVis) {
            VisUtil.initVisualization(problem, "Trajectory Tools ("+method.toString()+")", params.bgImageFile, params.timeStep/2);
            VisUtil.visualizeEarliestArrivalProblem(problem);
        }

        Disturbance disturbance = new Disturbance((float) params.disturbanceProb, params.disturbanceQuantum, params.disturbanceSeed, problem.nAgents());
        
        switch (method) {
            case ALLSTOP:
                solveTracking(problem, disturbance, TrackingAgent.TrackingMethod.ALLSTOP, params);
                break;
        
			case RMTRACK:
	            solveTracking(problem, disturbance, TrackingAgent.TrackingMethod.RMTRACK, params);
	            break;

	        case ORCA:
                solveORCA(problem, params);
                break;

            default:
                throw new RuntimeException("Unknown method");

        }
    }

	private static void solveTracking(final EarliestArrivalProblem problem, Disturbance disturbance,
                                      TrackingAgent.TrackingMethod trackingMethod, final Parameters params) {

        // find minimum lengths
        double[] baseTaskDurations = RPP.findBaseTaskDurations(problem);
        // find trajectories
        final EvaluatedTrajectory[] trajs = RPP.findCollisionFreeTrajs(problem, params.timeStep, params.maxTime);

        List<Agent> agents = new LinkedList<>();
        for (int i=0; i < problem.nAgents(); i++) {
            agents.add(i,
                    new TrackingAgent(i,
                        problem.getStart(i).toPoint2d(),
                        problem.getTarget(i).toPoint2d(),
                        problem.getBodyRadius(i),
                        problem.getMaxSpeed(i),
                        trajs[i],
                        disturbance,
                        trackingMethod,
                        (int) baseTaskDurations[i]));
        }

		// simulate execution
		simulate(problem, agents, params);
    } 
	
	private static void solveORCA(final EarliestArrivalProblem problem, final Parameters params) {
//        simulate(problem, new AgentFactory() {
//            @Override
//            public Agent createAgent(String name, int i, Point start, int nTasks,
//                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius, float speed) {
//
//				Agent agent = new ORCAAgent(name, start, nTasks, env, planningGraph, agentBodyRadius, speed, params.maxTime, params.timeStep, params.showVis, params.random);
//				return agent;
//            }
//        }, params);
    }    

    private static void simulate(final EarliestArrivalProblem problem, List<Agent> agents, final Parameters params) {
    	
    	simulationStartedAt = System.currentTimeMillis();
        initAgentVisualization(agents, params.timeStep);

        int SIMULATION_STEP_MS = 100;
        int SIMULATE_UNTIL = 600000;
        int simulatedTimeMs = 0;
        while (!allDone(agents) && simulatedTimeMs < SIMULATE_UNTIL){
            simulatedTimeMs += SIMULATION_STEP_MS;

            for (Agent agent: agents){
                agent.update(simulatedTimeMs, simulatedTimeMs+SIMULATION_STEP_MS, agents);
            }

            if (params.showVis) {
                try {
                    Thread.sleep(SIMULATION_STEP_MS);
                } catch (InterruptedException e) {
                }
            }
        }

        long baseTimeSum = 0;
    	long travelTimeSum = 0;
        long prolongSum = 0;
    	long prolongSumSq = 0;
        long makespanAbs = 0;
        long makespanProlong = 0;

        for (Agent agent : agents) {
            baseTimeSum += agent.baseTaskDuration;
            travelTimeSum += agent.travelTime;
            long prolongation = agent.travelTime - agent.baseTaskDuration;
            prolongSum += prolongation;
            prolongSumSq += prolongation * prolongation;

            if (agent.travelTime > makespanAbs)
                makespanAbs = agent.travelTime;

            if ((agent.travelTime - agent.baseTaskDuration) > makespanProlong)
                makespanProlong = (agent.travelTime - agent.baseTaskDuration);
		}
        
        long n = agents.size();
        
        long avgBaseTime = avg(baseTimeSum, n);
        long avgTravelTime = avg(travelTimeSum, n);
        long avgProlong = avg(prolongSum, n);
        long varProlong = sd(prolongSumSq, avgProlong, n);

        // status;dprob;dquant;dseed;avgBase;avgTravel;avgProlong;varProlong;makespanAbs;makespanRel

		printSummary(params.summaryPrefix, Status.SUCCESS, params.disturbanceProb, params.disturbanceQuantum, params.disturbanceSeed,
            avgBaseTime, avgTravelTime, avgProlong, varProlong, makespanAbs, makespanProlong);
    }
    
    private static boolean allDone(List<Agent> agents) {
        final double EPS = 0.9;
    	for (final Agent agent : agents) {
            agent.isAtGoal();
    		if (!agent.isAtGoal()) {
                return false;
            }
    	}
    	return true;
	}


	private static void initAgentVisualization(final List<Agent> agents, int timeStep) {
        // trajectories
        VisManager.registerLayer(
                KeyToggleLayer.create("t", true,
                        FastTrajectoriesLayer.create(new TrajectoriesProvider() {

                            @Override
                            public Trajectory[] getTrajectories() {
                                Trajectory[] trajsArr = new Trajectory[agents.size()];

                                for (int i = 0; i < trajsArr.length; i++) {
                                    Agent agent = agents.get(i);
                                    if (agent instanceof TrackingAgent) {
                                        trajsArr[i] = ((TrackingAgent) agent).getTrajectory();
                                    }
                                }
                                return trajsArr;
                            }
                        },new ColorProvider() {

                            @Override
                            public Color getColor(int i) {
                                return AgentColors.getColorForAgent(i);
                            }
                        }, 3, timeStep)));
		
        // positions
        VisManager.registerLayer(
        	KeyToggleLayer.create("b", true, 
        	LabeledCircleLayer.create(new LabeledCircleLayer.LabeledCircleProvider<tt.euclid2d.Point>() {

             @Override
             public Collection<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>> getLabeledCircles() {
                 LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>> list = new LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>>();
                 for (int i = 0; i < agents.size(); i++) {
                     tt.euclid2d.Point pos = agents.get(i).getPosition();

                     Color fillColor = AgentColors.getColorForAgent(i);
                     if (agents.get(i).wasDisturbed()) {
                         fillColor = Color.black;
                     }

                     Color textColor = Color.WHITE;

                     if (agents.get(i) instanceof TrackingAgent) {
                         TrackingAgent agent = (TrackingAgent) agents.get(i);
                         if (agent.isCurrentlyWaiting()) {
                             textColor = Color.DARK_GRAY;
                         }
                     }

                	 list.add(new LabeledCircleLayer.LabeledCircle<tt.euclid2d.Point>(pos,
                             (int) agents.get(i).getRadius(), "" + i  , AgentColors.getColorForAgent(i),
                             fillColor ,
                             textColor));
                 }

                 return list;
             }

         }, new tt.euclid2d.vis.ProjectionTo2d())));
 	}

    enum Status {SUCCESS, FAIL, TIMEOUT}

	static long avg(long sum, long n) {
		return sum / n;
	}
    /* status;dprob;dquant;dseed;avgBase;avgTravel;avgProlong;varProlong;makespanAbs;makespanRel */
    private static void printSummary(String prefix, Status status, double dprob, int dquant, int dseed,
                                     long avgBase, long avgTravel, long avgProlong, long varProlong, long makespanAbs,
                                     long makespanRel) {
        System.out.println(prefix +
                status.toString() + ";" +
                dprob + ";" +
                dquant + ";" +
                dseed + ";" +
                avgBase + ";" +
                avgTravel + ";" +
                avgProlong + ";" +
                varProlong + ";" +
                makespanAbs + ";" +
                makespanRel + ";");
    }

	static long sd(long sumSq, long mean, long n) {
		double var = (sumSq/n) - (mean*mean); 
		return (long) Math.round(Math.sqrt(var));
	}

}
