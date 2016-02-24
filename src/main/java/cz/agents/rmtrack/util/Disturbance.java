package cz.agents.rmtrack.util;

import java.util.Random;

public class Disturbance {

    private final boolean[][] disturbances;
    private final int[] offsets;
    float probability;
    int timeQuantum;
    int seed;


    public Disturbance(float probability, int timeQuantum, int seed, int nAgents) {
        this.probability = probability;
        this.timeQuantum = timeQuantum;
        this.seed = seed;

        Random random = new Random(seed);

        int len = 10000;
        disturbances = new boolean[nAgents][len];
        for (int i=0; i<nAgents; i++) {
            for (int t=0; t<len; t++)
            disturbances[i][t] = random.nextFloat() < probability;
        }

        offsets = new int[nAgents];
        for (int i=0; i<nAgents; i++) {
            offsets[i] = -random.nextInt(timeQuantum);
        }

    }

    public boolean isDisturbed(int i, int time){
        int tq = (time + offsets[i])/ timeQuantum;
        return disturbances[i][tq];
    }
}
