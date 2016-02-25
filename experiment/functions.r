library(plyr)
library(ggplot2)
library(RColorBrewer)
#install.packages('gridExtra')
library(gridExtra)

pd <- position_dodge(0)

load.and.preprocess <- function(env) {
  dir <- paste("instances/",env, sep="")
  runs <- read.csv(file=paste(dir, "/data.out.head", sep=""), head=TRUE, sep=";")
  runs <- runs[order(runs$instance, runs$alg),]

  maxagents <<- max(runs$nagents) + 3

  return(runs) 
}

avgtravel.vs.disturbance <- function(runs) {
  nrobots <- max(runs$nagents)
  avgbase <- mean(runs$avgBase, na.rm=TRUE)
  travel <- ddply(runs, .(dprob, alg), summarise,  
                  N = sum(!is.na(avgTravel)),
                  mean = mean(avgTravel, na.rm=TRUE),
                  sd = mean(varProlong, na.rm=TRUE),
                  se = sd / sqrt(N*max(nagents)*4))
  
  plot <- ggplot(travel, aes(x=dprob*100, y=mean/1000, color=alg, linetype=alg, shape=alg)) +
    geom_errorbar(aes(ymin=(mean-sd)/1000, ymax=(mean+sd)/1000), width=4, position=pd, size=0.5, alpha=0.5) +
    geom_line(size=1, position=pd)+ 
    geom_point(size=3, position=pd, fill="white")+   
    scale_y_continuous(limits=c(0,200),name="avg. travel time [s]") +
    scale_x_continuous(limits=c(0,50), name="disturbance [%]") +  
    scale_linetype_discrete(name="Method: ") +
    scale_color_discrete(name="Method: ") +
    scale_shape(name="Method: ") +
    geom_hline(yintercept=avgbase/1000, linetype='dotted', show_guide = TRUE, name='Move duration') +
    theme_bw() +
    ggtitle(paste("Avg. travel time to reach destination."))
}

##################### OLD CODE ########################

common.runs <- function(runs, algs) {
  solved.by.all <- unique(runs$instance)
  for (alg in algs) {
    solved.by.all <- intersect(solved.by.all, unique(runs[runs$alg==alg & runs$status=="SUCCESS", "instance"]))                              
  }
  
  common.runs <- runs[is.element(runs$instance, solved.by.all) & is.element(runs$alg,algs), ] 
  return(common.runs)
}

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



