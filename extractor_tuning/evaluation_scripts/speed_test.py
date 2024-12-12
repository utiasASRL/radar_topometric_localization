import os
import os.path as osp
import numpy as np
from collections import deque
import re


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


def pol_extracted_point_count(odom_folder, sequence):
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

def parse_number_from_name(name):
    name = name.replace((config+"_"), '')
    components = name.split("_")
    components = components[1:]

    parameters = []
    for param in components:
        if "f" in param:
            param = float(param.split("f")[0]+"."+param.split("f")[1])
        else:
            param = int(param)
        parameters.append(param)
    return parameters

def parse_seq_from_name(name):
    return name.split("_")[0]

def read_first_lines(filepath, num_lines=100):
    # Check if file exists
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File '{filepath}' does not exist.")
    
    # Initialize a list to store the first `num_lines` lines
    lines = []

    # Open the file for reading
    with open(filepath, 'r', encoding='utf-8') as file:
        # Read each line in the file
        for i, line in enumerate(file):
            # Append the line to the list
            lines.append(line)
            # Stop reading if we have reached the desired number of lines
            if i + 1 >= num_lines:
                break
    # Join the lines and return as a single string
    return ''.join(lines)
        
def get_icp_failure_count(file_name,sequence):
    ROOTDIR = os.environ['ROOTDIR']
    VTRRESULT = os.environ['VTRRESULT']
    VTRRROOT = os.environ['VTRRROOT']
    VTRRDATA = os.environ['VTRRDATA']
    SENSOR = "radar"
    VTRRRESULT = os.path.join(VTRRESULT,SENSOR)

    odom_log_path = os.path.join(VTRRRESULT,"detectors",config,"results",sequence,file_name,file_name)
    
    files = os.listdir(odom_log_path)

    log_files = [os.path.join(odom_log_path, file) for file in files if file.endswith('.log')]

    if not log_files:
        print(f"No .log files found in '{odom_log_path}'.")
        return True

    most_recent_file = max(log_files, key=os.path.getmtime)

    # Found 3576 radar data
    contents = read_first_lines(most_recent_file, num_lines=100)

    # Use regular expression to find the number in the format "Found X radar data"
    match = re.search(r"Found (\d+) radar data", contents)
    
    frame_count = 0
    # Extract and return the number if found
    if match:
        frame_count = int(match.group(1))-1
    else:
        raise ValueError("The pattern 'Found X radar data' was not found in the first lines of the file.")

    ICP_fail_count = 0
    last_frame_executed = 0
    with open(most_recent_file) as infile:
        for line in infile:
            if "ICP did not converge to the specified threshold" in line:
                ICP_fail_count += 1 
            if "Loading radar frame" in line:
                last_frame_executed = int(line.split(" ")[8])

    if last_frame_executed != frame_count:
        # subtract 1 becuase this frame failed
        last_frame_executed = last_frame_executed-1

    if last_frame_executed != 0:
        percent_converged = 100 * (last_frame_executed-ICP_fail_count) / last_frame_executed
        percent_executed = 100 * last_frame_executed / frame_count
    else:
        percent_converged = 0
        percent_executed = 0

    return percent_converged, percent_executed


def get_run_time(log_file):
    runtime = 0
    with open(log_file) as infile:
        for line in infile:
            if "Summarizing module: radar.navtech_extractor" in line:
                runtime = float(line.split(" ")[13][0:6])
    
    if runtime == 0:
        # print(log_file+" FAILED")
        return 0

    return runtime

def get_avg_pts(log_file):

    point_counts = []
    with open(log_file) as infile:
        for line in infile:
            if "Extracted " in line:
                point_counts.append(int(line.split(" ")[6]))

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

# # Save param file
# ODO_INPUT = SEQUENCE + "_" + config_type
# if val_1 is not None: 
#     if '.' in str(val_1): ODO_INPUT = ODO_INPUT+"_"+(str(val_1).replace(".", "f"))
#     else: ODO_INPUT = ODO_INPUT+"_"+str(val_1)
# if val_2 is not None:
#     if '.' in str(val_2): ODO_INPUT = ODO_INPUT+"_"+(str(val_2).replace(".", "f"))
#     else: ODO_INPUT = ODO_INPUT+"_"+str(val_2)
# if val_3 is not None:
#     if '.' in str(val_3): ODO_INPUT = ODO_INPUT+"_"+(str(val_3).replace(".", "f"))
#     else: ODO_INPUT = ODO_INPUT+"_"+str(val_3)

# destination = os.path.join(ROOTDIR,"speed_test_results","radar","detectors",config_type,"results",SEQUENCE,ODO_INPUT)




# "cacfar"
# configs =["bfar","kstrongest","cen2018","cen2019","oscfar","cago_cfar","caso_cfar","msca_cfar","tm_cfar","is_cfar","vi_cfar","cfear_kstrong"]
configs =["kstrongest","tm_cfar","is_cfar","vi_cfar","cfear_kstrong"]
# sequences = ["boreas-2020-12-04-14-00","boreas-2021-01-26-10-59"]#"boreas-2021-02-09-12-55"]

#NEW test
sequences = ['boreas-2021-11-02-11-16','boreas-2021-11-06-18-55']
# sequences=['boreas-2021-11-02-11-16', 'boreas-2021-11-06-18-55', 'boreas-2021-11-14-09-47', 'boreas-2021-11-16-14-10', 'boreas-2021-11-23-14-27', 'boreas-2021-11-28-09-18']



conifg_dict = {}

for config in configs:
    runtimes = []
    avg_pts = []
    for seq in sequences:
        results_base_dir  = os.path.join(ROOTDIR,"speed_test_results","radar","detectors",config,"results",seq)
        
        for folder in os.listdir(results_base_dir):
            full_path = os.path.join(results_base_dir, folder)
            if os.path.isdir(full_path) and folder.startswith(seq):
                sequence_with_config = folder
                break
        
        log_dir = os.path.join(results_base_dir,sequence_with_config,sequence_with_config)

        log_files = [os.path.join(log_dir, file) for file in os.listdir(log_dir) if file.endswith('.log')]

        if not log_files:
            print(f"No .log files found in '{log_dir}'.")
            break

        log_file = max(log_files, key=os.path.getmtime)

        runtime = get_run_time(log_file)
        if runtime == 0:
            print("config: ",config," on seq: ",seq, " is missing timing info")
        runtimes.append(runtime)
        avg_pts.append(get_avg_pts(log_file))
    
    conifg_dict[config] = [sum(runtimes)/len(runtimes),int(sum(avg_pts)/len(avg_pts))]

for config, res in conifg_dict.items():
    [runtime,avg_pts] = res
    print("Config: "+config+" Pre-Processing runtime: " + str(runtime) + " Average Number of points: " + str(avg_pts))