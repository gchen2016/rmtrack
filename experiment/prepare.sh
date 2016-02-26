#!/bin/bash

function generate_instance_set {
    denvxml=$1
    instancefolder=$2
    radius=$3
    gridedgelen=$4
    maxtime=$5
    agents=$6
    maxspeed="0.05"
    timestep=`echo "import math;print(int(math.ceil($gridedgelen/(2*$maxspeed))))" | python`

    echo "Preparing instanceset $instancesetname. Will use timestep $timestep."

    mkdir -p $instancefolder
    rm $instancefolder/*
    cp prepare.sh $instancefolder/

    instance=0
    for nagents in $agents
    do
        for seed in {1..5}
        do
            let instance=instance+1
            # create a problem instance file
            instancename="$instance"
            instancefile=$instancefolder/$instancename.xml

            ## ConflictGenerator
            java -XX:+UseSerialGC -cp solver.jar -Dlog4j.configuration="file:$PWD/log4j.custom" tt.jointeuclid2ni.probleminstance.generator.GenerateEAInstance -env $denvxml -nagents $nagents -radius $radius -maxspeed $maxspeed -seed $seed -sgnooverlap -outfile $instancefile

            algs="ALLSTOP RMTRACK"

            for alg in $algs
            do
                for dprob in 0 2 6 8 10 15 30 40 50
                do
                    summaryprefix="$instance;$nagents;$radius;$seed;$timestep;$maxtime;$alg;"
                    echo -method $alg -problemfile $instancefile -timestep $timestep -maxtime $maxtime -timeout $maxtime -dseed 1 -dprob $dprob -dquant 1000 -summaryprefix "$summaryprefix" >> $instancefolder/data.in
                done
            done

            echo Finished instance no $instance. Agents: $nagents. Seed: $seed.
        done
    done
    echo "instance;nagents;radius;seed;timestep;maxtime;alg;status;dprob;dquant;dseed;avgBase;avgTravel;avgProlong;varProlong;makespanAbs;makespanRel;" > $instancefolder/head
    echo Done. Created $instance instances at $envname environment. Instances stored in $instancefolder.
}

# ubremen
generate_instance_set d-envs/ubremen-r27-docks.xml instances/ubremen-r27 27 65 600000 "10 20 35" # up to 35

# warehouse
generate_instance_set d-envs/warehouse-r25-docks.xml instances/warehouse-r25 25 54 600000 "10 25 50"

# emptyhall
generate_instance_set d-envs/empty-hall-r25-docks.xml instances/empty-hall-r25 20 54 600000 "10 25 50"
