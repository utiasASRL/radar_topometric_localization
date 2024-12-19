import os
import os.path as osp
import numpy as np
from collections import deque
import re

def get_run_time(log_file):
    runtime = 0
    with open(log_file) as infile:
        for line in infile:
            if "Summarizing module: radar.pc_extractor" in line:
                runtime = float(line.split(" ")[13][0:6])
    
    if runtime == 0:
        return 0

    return runtime

def get_avg_pts(log_file):

    point_counts = []
    with open(log_file) as infile:
        for line in infile:
            if "Extracted " in line:
                point_counts.append(int(line.split(" ")[7]))

    avg_pts = sum(point_counts) / len(point_counts)
    return int(avg_pts)

ROOTDIR = os.environ['ROOTDIR']
SENSOR = "radar"
VTRRESULT = os.environ['VTRRESULT']
VTRRROOT = os.environ['VTRRROOT']
VTRRDATA = os.environ['VTRRDATA']

# Set results subfolder, VTRRESULT is set in setup_container.sh
VTRRRESULT = os.path.join(VTRRESULT,SENSOR)
os.environ['VTRRRESULT'] = VTRRRESULT

# configs =["bfar","kstrongest","cen2018","cen2019","oscfar","cago_cfar","caso_cfar","msca_cfar","tm_cfar","is_cfar","vi_cfar","cfear_kstrong"]
configs =["kstrongest"]

#NEW test
sequences = ['boreas-2021-11-14-09-47', 'boreas-2021-11-16-14-10', 'boreas-2021-11-23-14-27']

conifg_dict = {}

for config in configs:
    runtimes = []
    avg_pts = []
    for seq in sequences:
        results_base_dir  = os.path.join(VTRRRESULT,"detectors",config,"results",seq)
        
        for folder in os.listdir(results_base_dir):
            full_path = os.path.join(results_base_dir, folder)
            if os.path.isdir(full_path) and folder.startswith(seq):
                sequence_with_config = folder
        
                log_dir = os.path.join(results_base_dir,sequence_with_config,sequence_with_config)

                log_files = [os.path.join(log_dir, file) for file in os.listdir(log_dir) if file.endswith('.log')]

                if not log_files:
                    print(f"No .log files found in '{log_dir}'.")
                    break

                log_file = max(log_files, key=os.path.getmtime)

                runtime = get_run_time(log_file)
                points = get_avg_pts(log_file)
                parameters = sequence_with_config.split("_")[2:]
                if runtime == 0:
                    print("config: ",config," on seq: ",seq, " with parameters: ",parameters," is missing timing info")
                else:
                    print("config: ",config," on seq: ",seq, " with parameters: ",parameters," extracted ",points," points in ",runtime," seconds") 
                runtimes.append(runtime)
                avg_pts.append(points)
            
    conifg_dict[config] = [sum(runtimes)/len(runtimes),int(sum(avg_pts)/len(avg_pts))]

for config, res in conifg_dict.items():
    [runtime,avg_pts] = res
    print("Config: "+config+" Pre-Processing runtime: " + str(runtime) + " Average Number of points: " + str(avg_pts))