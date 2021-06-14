import argparse
import os
import sys
from datetime import datetime

if 'SUMO_HOME' in os.environ:
 tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
 sys.path.append(tools)
else:   
 sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import sumolib
from env import LearningType, SumoEnvironment

import pandas as pd
import matplotlib.pyplot as plt


if __name__ == '__main__':

  prs = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter,
                                description="""Launcher""")
  prs.add_argument("-net", dest="net", type=str, default='nets_data/Grid4x4/4x4_Webster.net.xml', help="Net definition xml file.\n")
  prs.add_argument("-route", dest="route", type=str, default='nets_data/Grid4x4/route_500.rou.xml', help="Route definition xml file.\n")
  prs.add_argument("-region", dest="region", type=str, default = None, help="Region definition json file.\n")
  prs.add_argument("-gui", action="store_true", default=False, help="Run with visualization on SUMO.\n")
  prs.add_argument("-s", dest="seconds", type=int, default=25000, required=False, help="Number of simulation seconds.\n")
  prs.add_argument("-w", dest="warmUp", type=int, default=0, help="Warm up time in seconds.\n")
  
  prs.add_argument("-l", "--learning", choices=["n", "a"] , default="a", required=False, help="Type of learning: n - no learning (fixed time), a - approximation.\n")
  prs.add_argument("-sl", "--saveLearning", action="store_true" , default=False, required=False, help="Save and Load Learning Weights")

  prs.add_argument("-ai", dest="alphaIntersec", type=float, default=0.0001, required=False, help="Intersection agents alpha learning rate.\n")
  prs.add_argument("-gi", dest="gammaIntersec", type=float, default=0.95, required=False, help="Intersection agents gamma discount rate.\n")

  prs.add_argument("-ar", dest="alphaRegion", type=float, default=0.000002, required=False, help="Region agents alpha learning rate.\n")
  prs.add_argument("-gr", dest="gammaRegion", type=float, default=0.95, required=False, help="Region agents gamma discount rate.\n")

  #prs.add_argument("-e", dest="epsilon", type=float, default=0.05, required=False, help="Epsilon.\n")
  #prs.add_argument("-me", dest="min_epsilon", type=float, default=0.005, required=False, help="Minimum epsilon.\n")
  #prs.add_argument("-d", dest="decay", type=float, default=1.0, required=False, help="Epsilon decay.\n")

  #prs.add_argument("-mingreen", dest="min_green", type=int, default=10, required=False, help="Minimum green time.\n")
  #prs.add_argument("-maxgreen", dest="max_green", type=int, default=30, required=False, help="Maximum green time.\n")
  
  prs.add_argument("-runs", dest="runs", type=int, default=1, help="Number of runs.\n")
  prs.add_argument("-v", action="store_true", default=False, help="Print learning tuple.\n")

  prs.add_argument("-i", dest="intersecList", type=str, nargs="+", default = list(), help="List of intersection IDs to watch during the simulation\n")
  prs.add_argument("-agents", dest="agentsInfo", action="store_true", default = False, help="Generate csvs for each agent\n")
  
  args = prs.parse_args()

  start_time = datetime.now()
  out_csv = "{}_{:02d}_{:02d}_{:02d}{:02d}".format(start_time.year, start_time.month, start_time.day, start_time.hour, start_time.minute)
  #out_csv = 'outputs/single-intersection/{}_alpha{}_gamma{}_eps{}_decay{}_reward{}'.format(experiment_time, args.alpha, args.gamma, args.epsilon, args.decay)

  expId = args.net.split('/')[1] + '_' + args.route.split('/')[2].split('.')[0]

  if args.region:
    print ()

  if args.learning == "n":
    learningtype = LearningType.NO_LEARNING
    out_csv += "_fixed"
  else:
    learningtype = LearningType.APPROXIMATION
    out_csv += "_learning_ai{}_gi{}".format(args.alphaIntersec, args.gammaIntersec)

  if args.region:
    expId += "_" + args.region.split('/')[2].split('.')[0]
    out_csv += "_ar{}_gr{}".format(args.alphaRegion, args.gammaRegion)

  # Save Learning
  if args.saveLearning:
    saveLearningDirName = "weights/" + expId + "/" + out_csv + "/"
    if not os.path.exists(saveLearningDirName):
      os.makedirs(saveLearningDirName)
  else:
    saveLearningDirName = None


  # Output -----
  outputDirName = "outputs/" + expId + "/" + out_csv + "/"
  if not os.path.exists(outputDirName):
    os.makedirs(outputDirName)
  out_csv = outputDirName + out_csv

  print ("AI: {}\tGI: {}\tAR: {}\tGR: {}\n".format(args.alphaIntersec, args.gammaIntersec, args.alphaRegion, args.gammaRegion))
  if args.intersecList:
    print ("List: {}\n".format(args.intersecList))

  # Launch -----
  env = SumoEnvironment(net_file=args.net,
                        route_file=args.route,
                        use_gui=args.gui,

                        simulationMaxSeconds = args.seconds,
                        warmUpTime = args.warmUp,

                        learningtype = learningtype,
                        saveLearning = args.saveLearning,
                        saveLearningDirName = saveLearningDirName,
                        printLearning=args.v,

                        alphaIntersec = args.alphaIntersec,
                        gammaIntersec = args.gammaIntersec,
                        alphaRegion = args.alphaRegion,
                        gammaRegion = args.gammaRegion,

                        regionsFileName = args.region,
                        intersectionList = args.intersecList,
                        agentsInfo = args.agentsInfo
                        )

  
  options = ['ave_wait', 'ave_rewards', 'total_veh_in_network', 'departed_veh', 'arrived_veh', 'startingTeleport_veh', 'endingTeleport_veh', 'ave_occupancy', 'ave_stopped', 'ave_travel_time', 'ave_mean_speed']

  optionsDFs = dict()

  for option in options:
    optionsDFs[option] = pd.DataFrame()

  for run in range(1, args.runs+1):
    env.run(run, outputDirName)
    aveWaitTime, run_df = env.save_csv(out_csv, run, ['step_time'] + options)
    env.close()

    for option in options:
      optionsDFs[option]['step_time'] = run_df['step_time']
      optionsDFs[option][option +'_run{}'.format(run)] = run_df[option]

  for option in options:
    optionDF = optionsDFs[option]

    optionDF.set_index('step_time', inplace=True)
    optionDF['mean'] = optionDF.mean(axis = 1)
    optionDF['std'] = optionDF.std(axis = 1)
    optionDF.to_csv(out_csv + '_allRuns_{}.csv'.format(option), decimal=',')
    optionDF.reset_index(inplace = True)

    optionDF.drop(optionDF.tail(2).index,inplace=True) # remove mean and std rows for the figure plots

    # First Plot Figure
    fig1 = plt.figure()
    # df.reset_index(inplace = True)
    plt.plot(optionDF['step_time'], optionDF['mean'], 'b-', label=option)
    plt.fill_between(optionDF['step_time'], optionDF['mean'] - optionDF['std'], optionDF['mean'] + optionDF['std'], color='b', alpha=0.2)
    plt.xlabel('step_time')
    plt.ylabel(option)
    plt.legend()
    fig1.savefig(out_csv + '_{}_mean.png'.format(option))
    plt.close(fig1)


    fig2 = plt.figure()
    for run in range(1, args.runs+1):
      label = option + '_run{}'.format(run)
      plt.plot(optionDF['step_time'], optionDF[label], label=label)
    plt.plot(optionDF['step_time'], optionDF['mean'], label='mean')
    plt.xlabel('step_time')
    plt.ylabel(option)
    plt.legend()
    fig2.savefig(out_csv + '_{}_allRuns.png'.format(option))
    plt.close(fig2)
   
    #plt.show()
  

  print("Executed in {}".format(datetime.now() - start_time))