library(plyr)
library(ggplot2)
library(RColorBrewer)
#install.packages('gridExtra')
library(gridExtra)

rmtrackColor <-"#E69F00"
rmtrackShape <- 1
allstopColor <- "#56B4E9"
allstopShape <- 2
orcaColor <- "#009E73"
orcaShape <- 3

pd <- position_dodge(0)

load.and.preprocess <- function(env) {
  dir <- paste("instances/",env, sep="")
  runs <- read.csv(file=paste(dir, "/data.out.head", sep=""), head=TRUE, sep=";")
  runs <- runs[order(runs$instance, runs$alg),]
  
  runs$travelTimeSum[runs$status != "SUCCESS"] <- NA
  runs$travelTimeSumSq[runs$status != "SUCCESS"] <- NA
  
  runs$spSum[runs$status != "SUCCESS"] <- NA
  runs$d0Sum[runs$status != "SUCCESS"] <- NA
  runs$lbSum[runs$status != "SUCCESS"] <- NA
  
  
  runs$prolongSpSum[runs$status != "SUCCESS"] <- NA
  runs$prolongSpSumSq[runs$status != "SUCCESS"] <- NA
  runs$prolongLBSum[runs$status != "SUCCESS"] <- NA
  runs$prolongLBSumSq[runs$status != "SUCCESS"] <- NA  
  
  maxagents <<- max(runs$nagents) + 3

  return(runs) 
}

common.rmtrack.orca.runs <- function(runs) {
  solved.by.rmtrack <- unique(runs[runs$alg=="RMTRACK" & runs$status == "SUCCESS", c("instance","dprob")])
  solved.by.orca <- unique(runs[runs$alg=="ORCA" & runs$status == "SUCCESS", c("instance","dprob")])
  common <- merge(solved.by.rmtrack, solved.by.orca)
  common.runs <- merge(common, runs[runs$alg=="ORCA" | runs$alg=="RMTRACK",])
  return(common.runs)
}

avgtravel.vs.disturbance <- function(runs) {
  subruns <- runs[runs$alg=="RMTRACK" | runs$alg=="ALLSTOP", ]
  nrobots <- max(subruns$nagents)
  
  travel <- ddply(subruns, .(dprob, alg), summarise,  
                  N = sum(!is.na(travelTimeSum)),
                  travelTimeSum = sum(travelTimeSum, na.rm=TRUE),
                  travelTimeSumSq = sum(travelTimeSumSq, na.rm=TRUE),
                  lbSum = sum(lbSum, na.rm=TRUE),
                  spSum = sum(spSum, na.rm=TRUE),
                  prolongSpSum = sum(prolongSpSum, na.rm=TRUE),
                  prolongSpSumSq = sum(prolongSpSumSq, na.rm=TRUE),
                  prolongLBSum = sum(prolongLBSum, na.rm=TRUE),
                  prolongLBSumSq = sum(prolongLBSumSq, na.rm=TRUE))
                  
  travel$travelTimeAvg <- travel$travelTimeSum/(travel$N*nrobots)
  travel$travelTimeSd <- sqrt((travel$travelTimeSumSq/(travel$N*nrobots)) - travel$travelTimeAvg^2)
  
  travel$meanSp <- travel$spSum / (travel$N*nrobots)
  travel$meanLb <- travel$lbSum / (travel$N*nrobots)
  
  travel$prolongSpAvg <- travel$prolongSpSum/(travel$N*nrobots)
  travel$prolongSpSd <- sqrt((travel$prolongSpSumSq/(travel$N*nrobots)) - travel$prolongSpAvg^2)
  
  travel$prolongLBAvg <- travel$prolongLBSum/(travel$N*nrobots)
  travel$prolongLBSd <- sqrt((travel$prolongLBSumSq/(travel$N*nrobots)) - travel$prolongLBAvg^2)
  
  travel$mean <-travel$travelTimeAvg
  travel$sd <- travel$prolongLBSd
  
  #dprob <- unique(travel$dprob)
  
  travel$mean[travel$alg=="ALLSTOP" & travel$N==0] <- 600000
  travel$sd[travel$alg=="ALLSTOP" & travel$N==0] <- 0
  
  plot <- ggplot(travel, aes(x=dprob*100, y=mean/1000, color=alg, shape=alg)) +
    geom_hline(yintercept=max(travel[travel$alg=="RMTRACK","meanSp"], na.rm=TRUE)/1000, linetype='dotted', size=1, color="#888888") +
    geom_line(data=travel[travel$alg=="RMTRACK",], mapping=aes(x=dprob*100, y=meanLb/1000), size=1, color="#555555", linetype="dashed") +
    geom_errorbar(aes(ymin=(mean-sd)/1000, ymax=(mean+sd)/1000), width=4, position=pd, size=0.7, alpha=0.5) +
    geom_line(size=1, position=pd) +
    #geom_line(size=0.5, mapping=aes(y=meanLb/1000), color="red") + 
    geom_point(size=3, position=pd, fill="white") +   
    scale_y_continuous(name="avg. travel time [s]") +
    scale_x_continuous(name="disturbance intensity [%]", breaks=c(0,25,50)) +  
    #scale_color_discrete(name="Method: ") +
    #scale_shape(name="Method: ") +
    #geom_hline(yintercept=avgbase/1000, linetype='dotted', show_guide = TRUE, name='Move duration') +
    #scale_linetype_manual(values = c("data"="solid","bound"="dotted")) +
    scale_color_manual(values = c("ALLSTOP"=allstopColor,"RMTRACK"=rmtrackColor), name="Method: ") +
    scale_shape_manual(values = c("ALLSTOP"=allstopShape,"RMTRACK"=rmtrackShape), name="Method: ") +
    theme_bw() +
    coord_cartesian(ylim = c(-20, 220), xlim=c(-5, 55)) +
    ggtitle(paste("Avg. travel time (", nrobots, " rob.)", sep=""))
    
    return(plot)
}

avgtravel.vs.disturbance.common.only <- function(runs) {
  subruns <- common.rmtrack.orca.runs(runs)
  nrobots <- max(subruns$nagents)
  
  travel <- ddply(subruns, .(dprob, alg), summarise,  
                  N = sum(!is.na(travelTimeSum)),
                  travelTimeSum = sum(travelTimeSum, na.rm=TRUE),
                  travelTimeSumSq = sum(travelTimeSumSq, na.rm=TRUE),
                  spSum = sum(spSum, na.rm=TRUE),
                  prolongSpSum = sum(prolongSpSum, na.rm=TRUE),
                  prolongSpSumSq = sum(prolongSpSumSq, na.rm=TRUE))
  
  travel$travelTimeAvg <- travel$travelTimeSum/(travel$N*nrobots)
  travel$travelTimeSd <- sqrt((travel$travelTimeSumSq/(travel$N*nrobots)) - travel$travelTimeAvg^2)
  
  travel$meanSp <- travel$spSum / (travel$N*nrobots)
  
  travel$prolongSpAvg <- travel$prolongSpSum/(travel$N*nrobots)
  travel$prolongSpSd <- sqrt((travel$prolongSpSumSq/(travel$N*nrobots)) - travel$prolongSpAvg^2)
  
  travel$mean <-travel$travelTimeAvg
  travel$sd <- travel$prolongSpSd
  
  
  travel$mean[travel$N < 5] <- NA
  travel$sd[travel$N < 5] <- 0
  
  plot <- ggplot(travel, aes(x=dprob*100, y=mean/1000, color=alg, shape=alg)) +
    geom_hline(yintercept=max(travel[travel$alg=="RMTRACK","meanSp"], na.rm=TRUE)/1000, linetype='dotted', size=1, color="#888888") +
    geom_errorbar(aes(ymin=(mean-sd)/1000, ymax=(mean+sd)/1000), width=4, position=pd, size=0.5, alpha=0.5) +
    geom_line(size=1, position=pd) +
    geom_point(size=3, position=pd, fill="white") +   
    scale_y_continuous(name="avg. travel time [s]") +
    scale_x_continuous(limits=c(-2, 52), name="disturbance intensity [%]", breaks=c(0,25,50)) +  
    scale_color_manual(values = c("ORCA"=orcaColor,"RMTRACK"=rmtrackColor), name="Method: ") +
    scale_shape_manual(values = c("ORCA"=orcaShape,"RMTRACK"=rmtrackShape), name="Method: ") +
    theme_bw() +
    coord_cartesian(ylim = c(-20, 220), xlim=c(-5, 55)) +
    ggtitle(paste("Avg. travel time (", nrobots, " rob.)", sep=""))
  
  return(plot)
}

successrate.vs.disturbance <- function(runs) {
  subruns <- runs[runs$alg=="RMTRACK" | runs$alg=="ORCA", ]
  nrobots <- max(subruns$nagents)
  successrate <- ddply(subruns, .(dprob, alg), summarise,                     
                       successrate = sum(status=="SUCCESS") / length(status)
  )
  
  plot <- ggplot(successrate, aes(dprob*100, successrate*100, color=alg, shape=alg)) + 
    geom_point(size=3) + geom_line(size=1) +
    scale_y_continuous(limits=c(0,100), name=("instances solved [%]")) +  
    scale_x_continuous(limits=c(-2,52), name=("disturbance intensity [%]"), breaks=c(0,25,50)) +
    scale_color_manual(values = c("ORCA"=orcaColor,"RMTRACK"=rmtrackColor), name="Method: ") +
    scale_shape_manual(values = c("ORCA"=orcaShape,"RMTRACK"=rmtrackShape), name="Method: ") +
    coord_cartesian(ylim = c(-10, 110), xlim=c(-5, 55)) +
    theme_bw() +
    ggtitle(paste("Success rate (", nrobots, " rob.)", sep=""))
  
  return(plot)
}




##################### OLD CODE ########################


successrate.nagents <- function(runs) {
  #xbreaks <- unique(runs$nagents)
  successrate <- ddply(runs, .(nagents, alg), summarise,                     
                       successrate = sum(status=="SUCCESS") / length(unique(instance))
  )
  
  plot <- ggplot(successrate, aes(nagents, successrate*100, color=alg, shape=alg)) + 
    geom_point(size=3) + geom_line(size=1) +
    scale_y_continuous(limits=c(0,100), name=("Instances solved [%]")) +  
    scale_x_continuous(limits=c(0,maxagents), name=("No of robots")) +
    ggtitle("Success rate") +
    theme_bw()
  
  return(plot)
}

avgtaskprolong.vs.nagents <- function(runs) {  
  
  avgbase <- mean(runs$avgBase, na.rm=TRUE)
  
  travel <- ddply(runs, .(nagents, alg), summarise,  
                    N = sum(!is.na(avgProlongR)),
                    mean = mean(avgProlongR, na.rm=TRUE),
                    sd = mean(varProlongR, na.rm=TRUE),
                    se = sd / sqrt(N*max(nagents)*4))
  
  travelT <- ddply(runs[runs$alg=="COBRA",], .(nagents, alg), summarise,  
                  N = sum(!is.na(avgProlongT)),
                  mean = mean(avgProlongT, na.rm=TRUE),
                  sd = mean(varProlongT, na.rm=TRUE),
                  se = sd / sqrt(N*max(nagents)*4))
                   
  travelPlanning <- ddply(runs, .(nagents, alg), summarise,  
                  N = sum(!is.na(avgPWindow)),
                  mean = mean(avgPWindow, na.rm=TRUE),
                  sd = mean(varPWindow, na.rm=TRUE),
                  se = sd / sqrt(N*max(nagents)*4))                   
  
 
  travelPlanning$mean <- travelPlanning$mean + travel$mean
  travelPlanning$sd <- NA
  
  travel$type <- "travel only"
  travelT$type <- "travel (t) only "
  travelPlanning$type <- "planning + travel"
  
  combined <- rbind(travel, travelPlanning[travelPlanning$alg=="COBRA",])
  combined$type <- factor(combined$type, c("travel only", "planning + travel"))
  
  plot <- ggplot(combined, aes(x=nagents, y=mean/1000, color=alg, linetype=type, shape=alg)) +
    geom_errorbar(aes(ymin=(mean-sd)/1000, ymax=(mean+sd)/1000), width=4, position=pd, size=0.5, alpha=0.5) +
    #geom_errorbar(aes(ymin=(mean-se)/1000, ymax=(mean+se)/1000), width=4, position=pd, size=1, alpha=1) +
    geom_line(size=1, position=pd)+ 
    geom_point(size=3, position=pd, fill="white")+   
    #scale_y_continuous(limits=c(min(combined$mean-combined$sd, na.rm=TRUE)/1000, max(combined$mean+combined$sd, na.rm=TRUE)/1000), name="avg. prolongation [s]") +
    scale_y_continuous(name="avg. prolongation [s]") +
    scale_x_continuous(limits=c(0,maxagents), name="no of robots [-]") +  
    scale_linetype_discrete(name="") +
    scale_color_discrete(name="Method: ") +
    scale_shape(name="Method: ") +
    theme_bw() +
    ggtitle(paste("Avg. prolongation  of a relocation task (avg. ", round(avgbase/1000),  "s long)\n due to collision avoidance"))
  
  return(plot)
}

get.legend<-function(plot){
  tmp <- ggplotGrob(plot)
  leg <- which(sapply(tmp$grobs, function(x) x$name) == "guide-box")
  legend <- tmp$grobs[[leg]]
  return(legend)
}



