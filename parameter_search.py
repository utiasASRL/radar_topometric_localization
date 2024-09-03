import argparse
import ruamel.yaml
import os
import os.path as osp
import numpy as np
import time
from datetime import datetime
import subprocess
import csv
import external.vtr_testing_radar.src.vtr_testing_radar.script.boreas_generate_odometry_result as boreas_generate_odometry_result
from pyboreas.eval.odometry import eval_odom, eval_odom_vel
from collections import deque
import re
import signal

just_eval = False
eval_vel = True
dense_logging = False
max_process_count = 30
max_points = 2500

processes = []
result_dirs = []
total_process_count = 0
curr_process_count = 0
finished_process_count = 0


def read_last_lines(filepath, num_lines=100):
    # Check if file exists
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File '{filepath}' does not exist.")

    # Open the file for reading
    with open(filepath, 'r', encoding='utf-8') as file:
        # Initialize a deque to store the last `num_lines` lines
        last_lines = deque(maxlen=num_lines)
        
        # Read each line in the file and store in the deque
        for line in file:
            last_lines.append(line)
        
        # Join the lines and return as a single string
        return ''.join(last_lines)


def pol_extracted_point_count(curr_process, odom_folder, sequence):
    ROOTDIR = os.environ['ROOTDIR']
    VTRRESULT = os.environ['VTRRESULT']
    VTRRROOT = os.environ['VTRRROOT']
    VTRRDATA = os.environ['VTRRDATA']
    VTRRRESULT = os.path.join(VTRRESULT,args.sensor)
    
    odom_log_path = os.path.join(VTRRRESULT,"detectors",config,"results",sequence,odom_folder,odom_folder)
    # odom_log_path = os.path.join(VTRRRESULT,odom_folder,odom_folder)

    files = os.listdir(odom_log_path)

    log_files = [os.path.join(odom_log_path, file) for file in files if file.endswith('.log')]

    if not log_files:
        print(f"No .log files found in '{odom_log_path}'.")
        return True

    most_recent_file = max(log_files, key=os.path.getmtime)
    contents = read_last_lines(most_recent_file, num_lines=100)

    pattern = r'Extracted\s+(\d+)'
    # Search for the pattern in the content
    match = re.search(pattern, contents)
    integer_value = 0
    # If match is found, extract the integer value
    if match:
        integer_value = int(match.group(1))

    if integer_value > max_points:
        return True

    return False

def run_test(config_data, MODE, SENSOR, SEQUENCE, PARAM_FILE, config_type, val_1=None, val_2=None, val_3=None):
    global just_eval
    global max_process_count
    global processes
    global result_dirs
    global total_process_count
    global curr_process_count
    global finished_process_count

    ROOTDIR = os.environ['ROOTDIR']
    VTRRESULT = os.environ['VTRRESULT']
    VTRRROOT = os.environ['VTRRROOT']
    VTRRDATA = os.environ['VTRRDATA']

    # Set results subfolder, VTRRESULT is set in setup_container.sh
    VTRRRESULT = os.path.join(VTRRESULT,SENSOR)
    os.environ['VTRRRESULT'] = VTRRRESULT

    # Save param file
    ODO_INPUT = SEQUENCE + "_" + config_type
    if val_1 is not None: 
        if '.' in str(val_1): ODO_INPUT = ODO_INPUT+"_"+(str(val_1).replace(".", "f"))
        else: ODO_INPUT = ODO_INPUT+"_"+str(val_1)
    if val_2 is not None:
        if '.' in str(val_2): ODO_INPUT = ODO_INPUT+"_"+(str(val_2).replace(".", "f"))
        else: ODO_INPUT = ODO_INPUT+"_"+str(val_2)
    if val_3 is not None:
        if '.' in str(val_3): ODO_INPUT = ODO_INPUT+"_"+(str(val_3).replace(".", "f"))
        else: ODO_INPUT = ODO_INPUT+"_"+str(val_3)

    destination = os.path.join(VTRRRESULT,"detectors",config_type,"results",SEQUENCE,ODO_INPUT)
    # destination = os.path.join(VTRRRESULT,"detectors",config_type,SEQUENCE,ODO_INPUT)

    if not just_eval:
        try:
            os.makedirs(destination)
            # print(f"Directory '{destination}' created successfully (including parents).")
        except OSError as error:
            print(f"Failed to create directory '{destination}': {error}")

    SAVE_CONFIG=SENSOR+"_"+MODE+"_config.yaml"

    param_file_destination = os.path.join(destination,SAVE_CONFIG)

    # Create config file
    if not just_eval:
        open(param_file_destination, 'a').close()
        with open(param_file_destination, 'w') as new_config:
            yaml.dump(config_data, new_config)

    # # Call corresponding script from vtr_testing_radar
    # script_path = os.path.join(ROOTDIR,"test_odometry_alt.sh")
    # arguments =[ODO_INPUT, param_file_destination, destination, SEQUENCE, "&>/dev/null", "&"]
    # command = ['bash', script_path] + arguments

    command_init = ["bash" ,VTRRROOT+"/install/setup.bash"]
    command = ["ros2", "run","vtr_testing_radar","vtr_testing_radar_boreas_odometry", 
    "--ros-args", "-p", "use_sim_time:=true", "-r", "__ns:=/vtr",
    "--params-file",param_file_destination,
    "-p","data_dir:="+destination+"/"+ODO_INPUT,
    "-p", "odo_dir:="+VTRRDATA+"/"+SEQUENCE]

    if curr_process_count >= max_process_count and not just_eval:
        curr_head_proc = processes[finished_process_count]
        curr_odo_input = result_dirs[finished_process_count]
        print("Waiting; # of processes running:",curr_process_count, " Number of finished process:",finished_process_count, " total count: ", total_process_count)
        while curr_head_proc.poll() is None:
            halt_proc = pol_extracted_point_count(curr_head_proc,curr_odo_input, SEQUENCE)
            # if halt_proc:
            #     time.sleep(0.5)
            #     curr_head_proc.terminate()
            #     time.sleep(2)
            #     if curr_head_proc.poll() is None:
            #         curr_head_proc.kill()
            #         curr_head_proc.communicate()
            #     break
            # time.sleep(5)

            if halt_proc:
                time.sleep(0.5)
                # Kill the entire process group
                os.killpg(os.getpgid(curr_head_proc.pid), signal.SIGTERM)
                time.sleep(2)
                if curr_head_proc.poll() is None:
                    os.killpg(os.getpgid(curr_head_proc.pid), signal.SIGKILL)
                curr_head_proc.communicate()

        finished_process_count = finished_process_count + 1
        curr_process_count = curr_process_count -  1

    if not just_eval: 
        process_init = subprocess.Popen(command_init, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        process_init.wait()
        process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,preexec_fn=os.setsid)
        curr_process_count = curr_process_count + 1
        total_process_count = total_process_count + 1
        time.sleep(2)


    else: process = 1

    return process, ODO_INPUT


def run_eval(processes, result_dirs, SENSOR, seq, config):
    global just_eval
    global max_process_count
    # global processes
    # global result_dirs
    global curr_process_count
    global total_process_count
    global finished_process_count

    ROOTDIR = os.environ['ROOTDIR']
    VTRRESULT = os.environ['VTRRESULT']
    VTRRROOT = os.environ['VTRRROOT']
    VTRRDATA = os.environ['VTRRDATA']

    # Set results subfolder, VTRRESULT is set in setup_container.sh
    VTRRRESULT = os.path.join(VTRRESULT,SENSOR)
    os.environ['VTRRRESULT'] = VTRRRESULT

    results = []
    for count, process in enumerate(processes):
        result_dir = result_dirs[count]

        # path = os.path.join(VTRRRESULT, result_dir)
        path = os.path.join(VTRRRESULT,"detectors",config,"results",seq,result_dir)

        if not just_eval and process.poll() is None:
            print("Waiting; # of processes running:",curr_process_count, " Number of finished process:",finished_process_count)
            while process.poll() is None:
                halt_proc = pol_extracted_point_count(process,result_dir, seq)
                # if halt_proc: 
                #     time.sleep(0.5)
                #     process.terminate()
                #     time.sleep(2)
                #     if process.poll() is None:
                #         process.kill()
                #         process.communicate()
                # time.sleep(5)
                if halt_proc:
                    time.sleep(0.5)
                    # Kill the entire process group
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    time.sleep(2)
                    if process.poll() is None:
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    process.communicate()
                    
            finished_process_count = finished_process_count + 1
            curr_process_count = curr_process_count -  1

        boreas_generate_odometry_result.main(dataset_dir=VTRRDATA, result_dir=path, velocity=eval_vel)


        pred = os.path.join(path,"odometry_result/")
        try:
            curr_t_err, curr_r_err, curr_t_re_rmse, curr_t_re_rmse_99f9 = eval_odom(pred=pred,gt=VTRRDATA,radar=True)
            results.append([result_dir, curr_t_err, curr_r_err, curr_t_re_rmse, curr_t_re_rmse_99f9])
            if eval_vel:
                vel_pred = os.path.join(path,"odometry_vel_result/")
                eval_odom_vel(vel_pred, gt=VTRRDATA,radar=True)
                
        except:
            print("Failed to evaluate "+result_dir)
            results.append([result_dir, 0.0, 0.0, 0.0, 0.0])

    csv_file_name = str(datetime.today()).split()
    csv_file_name = seq+"_"+config+"_"+str(csv_file_name[0])+"_"+str(csv_file_name[1])+'.csv'
    # csv_file = os.path.join(VTRRRESULT,csv_file_name)
    
    if not os.path.exists(os.path.join(VTRRRESULT,"detectors",config,"data",seq)):
        os.makedirs(os.path.join(VTRRRESULT,"detectors",config,"data",seq))

    csv_file = os.path.join(VTRRRESULT,"detectors",config,"data",seq,csv_file_name)


    open(csv_file, 'a').close()

    with open(csv_file, 'w', newline='') as file_ref:
        file = csv.writer(file_ref, delimiter=' ',
                                quotechar='|', quoting=csv.QUOTE_MINIMAL)
        for result_dir, curr_t_err, curr_r_err, curr_t_re_rmse, curr_t_re_rmse_99f9 in results:
            file.writerow([result_dir, float(curr_t_err), float(curr_r_err), float(curr_t_re_rmse), float(curr_t_re_rmse_99f9)])


parser = argparse.ArgumentParser()

# Which config is being used
parser.add_argument("--config",default="")
# Which parameter file to use
parser.add_argument("--config_path",default="")
# Which mode Odometry or Localization
parser.add_argument("--mode",default="")
# Which sensor radar or lidar
parser.add_argument("--sensor",default="radar")
# Which sequence
parser.add_argument("--seq",default="")

args = parser.parse_args()
config = args.config
config_path = args.config_path
mode = args.mode
sensor = args.sensor
seq = args.seq

yaml = ruamel.yaml.YAML(typ='rt')
yaml.default_flow_style = False
yaml.preserve_quotes = True
yaml.allow_duplicate_keys = True
yaml.explicit_start = True

with open('detector_parameters.yaml', 'r') as yaml_file:
    data = yaml.load(yaml_file)

with open(config_path, 'r') as config_file:
    config_data = yaml.load(config_file)


# Add in extractor logs to get point count
updated_tactics = config_data['/**']['ros__parameters']['log_enabled']
if "radar.navtech_extractor" not in updated_tactics:
    updated_tactics.append("radar.navtech_extractor")
    config_data['/**']['ros__parameters']['log_enabled'] = updated_tactics

if dense_logging == False:
    sparse_tactics = ["radar.pipeline","tactic","tactic.module","radar.navtech_extractor"]
    config_data['/**']['ros__parameters']['log_enabled'] = sparse_tactics


# Disable visualization to prevent parallel runs from overflowing ROS
config_data['/**']['ros__parameters']['tactic']['visualize'] = False
config_data['/**']['ros__parameters']['preprocessing']['conversion']['landmark_extraction']['visualize'] = False
config_data['/**']['ros__parameters']['preprocessing']['filtering']['visualize'] = False
config_data['/**']['ros__parameters']['odometry']['mcransac']['visualize'] = False
config_data['/**']['ros__parameters']['odometry']['icp']['visualize'] = False
config_data['/**']['ros__parameters']['odometry']['mapping']['visualize'] = False
config_data['/**']['ros__parameters']['localization']['recall']['visualize'] = False


# Set yaml to use the correct config
config_data['/**']['ros__parameters']['preprocessing']['conversion']['detector'] = config

config_settings = data['/**']['detector'][config]

parameter_names = [None, None, None]
for count, parameter in enumerate(config_settings["parameters"]):parameter_names[count]=parameter

parameter_1 = config_settings["parameters"][parameter_names[0]]
parameter_1_range = np.arange(parameter_1["min"], parameter_1["max"] + parameter_1["step"], parameter_1["step"])
# print("HERE")
# print(parameter_1_range)
# sleep(100000)
for val_1 in parameter_1_range:
    val_1 = type(val_1.item())(val_1)
    if isinstance(val_1, float):
        val_1 = round(val_1, 3)
    if parameter_names[1] == None:
        # Change parameters in yaml
        config_data['/**']['ros__parameters']['preprocessing']['conversion'][config][parameter_names[0]] = val_1
        
        process, result = run_test(config_data=config_data, MODE=mode, SENSOR=sensor, SEQUENCE=seq, PARAM_FILE=config_path, config_type=config, val_1=val_1)  
        processes.append(process)
        result_dirs.append(result)
        continue
    
    parameter_2 = config_settings["parameters"][parameter_names[1]]
    parameter_2_range = np.arange(parameter_2["min"], parameter_2["max"] + parameter_2["step"], parameter_2["step"])

    # print("HERE")
    # print(parameter_2_range)
    # sleep(100000)
    for val_2 in parameter_2_range:
        val_2 = type(val_2.item())(val_2)
        if isinstance(val_2, float):
            val_2 = round(val_2, 3)

        if parameter_names[2] == None:
            # Change parameters in yaml
            config_data['/**']['ros__parameters']['preprocessing']['conversion'][config][parameter_names[0]] = val_1
            config_data['/**']['ros__parameters']['preprocessing']['conversion'][config][parameter_names[1]] = val_2

            process, result = run_test(config_data=config_data, MODE=mode, SENSOR=sensor, SEQUENCE=seq, PARAM_FILE=config_path, config_type=config, val_1=val_1, val_2=val_2)  
            processes.append(process)
            result_dirs.append(result)
            continue
        
        parameter_3 = config_settings["parameters"][parameter_names[2]]
        parameter_3_range = np.arange(parameter_3["min"], parameter_3["max"] + parameter_3["step"], parameter_3["step"])
        
        for val_3 in parameter_3_range:
            val_3 = type(val_3.item())(val_3)
            if isinstance(val_3, float):
                val_3 = round(val_3, 3)

            # Change parameters in yaml
            config_data['/**']['ros__parameters']['preprocessing']['conversion'][config][parameter_names[0]] = val_1
            config_data['/**']['ros__parameters']['preprocessing']['conversion'][config][parameter_names[1]] = val_2
            config_data['/**']['ros__parameters']['preprocessing']['conversion'][config][parameter_names[2]] = val_3

            process, result = run_test(config_data=config_data, MODE=mode, SENSOR=sensor, SEQUENCE=seq, PARAM_FILE=config_path, config_type=config, val_1=val_1, val_2=val_2, val_3=val_3)  
            processes.append(process)
            result_dirs.append(result)
            
            # Change parameters in yaml
            #run bash run in parallel script

time.sleep(1)
print("Triggered batch of processes using "+ config + " on sequence " +seq+". Waiting...")

run_eval(processes, result_dirs, SENSOR=sensor, seq=seq, config=config)

print("Done "+ config + ", " +seq+" batches!")