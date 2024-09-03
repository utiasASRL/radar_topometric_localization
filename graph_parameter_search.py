import matplotlib.pyplot as plt
from matplotlib import cm
import os
import fnmatch
import pandas as pd
import random
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import re


ROOTDIR = os.environ['ROOTDIR']
VTRRESULT = os.environ['VTRRESULT']
VTRRROOT = os.environ['VTRRROOT']
VTRRDATA = os.environ['VTRRDATA']
SENSOR = "radar"

# Set results subfolder, VTRRESULT is set in setup_container.sh
VTRRRESULT = os.path.join(VTRRESULT,SENSOR)
os.environ['VTRRRESULT'] = VTRRRESULT

graph_it = True
avg_1d = True
use_old_data = False
graph_mode = "3d" #"3d"
# config = "modified_cacfar_power"
# config = "caso_cfar"
# config = "cago_cfar"
config = "bfar_pure"
# config = "oscfar"
# config = "tm_cfar"
# config = "msca_cfar"
# config = "is_cfar"
# config = "vi_cfar"
# config = "msca_cfar"
# config = "kstrongest"
# config = "cen2018"
# config = "cen2019"
# config = "cfear_kstrong"


# sequences = ['boreas-2020-11-26-13-58', 'boreas-2021-01-26-10-59', 'boreas-2021-03-09-14-23']
# sequences = ['boreas-2020-11-26-13-58']
# boreas-2020-12-01-13-26
# boreas-2021-03-02-13-38
# boreas-2021-04-29-15-55
# boreas-2021-06-17-17-52
# sequences = ['boreas-2021-06-17-17-52']

# #TRAINING OLD
# sequences = ['boreas-2020-12-01-13-26', 'boreas-2021-03-02-13-38', 'boreas-2021-04-29-15-55','boreas-2021-06-17-17-52', 'boreas-2021-09-07-09-35','boreas-2021-08-05-13-34']

# sequences = ['boreas-2021-03-02-13-38','boreas-2021-04-29-15-55']
# 'boreas-2020-12-01-13-26', 'boreas-2021-03-02-13-38', 'boreas-2021-06-17-17-52', 'boreas-2021-04-29-15-55']

#TESTING OLD
# sequences = ['boreas-2020-12-04-14-00','boreas-2021-01-26-10-59','boreas-2021-02-09-12-55','boreas-2021-03-09-14-23','boreas-2021-06-29-18-53','boreas-2021-09-08-21-00']

# #TRAINING NEW
sequences=['boreas-2021-10-05-15-35','boreas-2021-10-15-12-35']#,'boreas-2021-10-22-11-36','boreas-2021-10-26-12-35']

# sequences=['boreas-2021-10-22-11-36','boreas-2021-10-26-12-35']
# sequences = ['boreas-2021-10-15-12-35']
# sequences=['boreas-2021-10-22-11-36','boreas-2021-10-26-12-35'] 

# # NEW Testing
# sequences=['boreas-2021-11-02-11-16', 'boreas-2021-11-06-18-55', 'boreas-2021-11-14-09-47', 'boreas-2021-11-16-14-10', 'boreas-2021-11-23-14-27', 'boreas-2021-11-28-09-18']


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


sequence_df = []

for count,seq in enumerate(sequences):
    path_to_save = os.path.join(VTRRRESULT,"detectors",config,"parameter_graphs")
    
    csv_prefix = seq+"_"+config

    # matching_files= [os.path.join(VTRRRESULT,"boreas-2020-11-26-13-58_bfar_pure_2024-07-05_17:18:14.808873.csv")]

    matching_files = []
    
    if use_old_data:
        for file in os.listdir(os.path.join(VTRRRESULT,"detectors",config,"data","old_data",seq)):
            if fnmatch.fnmatch(file, f'*{csv_prefix}*.csv'):
                matching_files.append(os.path.join(VTRRRESULT,"detectors",config,"data","old_data",seq,file))

    else:
        for file in os.listdir(os.path.join(VTRRRESULT,"detectors",config,"data",seq)):
            if fnmatch.fnmatch(file, f'*{csv_prefix}*.csv'):
                matching_files.append(os.path.join(VTRRRESULT,"detectors",config,"data",seq,file))


    # Read csv files and convert to pandas table
    dfs = []
    for file_path in matching_files:
        if file_path.endswith(".csv"):  
            df = pd.read_csv(file_path, sep=' ', names=["file","translation", "rotation", "rmse", "rmse_99f9","convergence_rates","execution_rates"])
            if df['convergence_rates'].isna().any() or df['execution_rates'].isna().any():
                seq_name = file_path.split("/")[-2]
                for index,row in df.iterrows():
                    filename = row["file"]
                    percent_converged, percent_executed = get_icp_failure_count(filename,seq_name)
                    df.at[index,'convergence_rates'] = percent_converged
                    df.at[index,'execution_rates'] = percent_executed
                df.to_csv(file_path, sep=' ',header=False,index=False)

            dfs.append(df)

    combined_df = pd.concat(dfs, ignore_index=True)

    combined_df['parameters'] = combined_df['file'].apply(parse_number_from_name)
    combined_df['seq'] = combined_df['file'].apply(parse_seq_from_name)

    sequence_df.append(combined_df)


sequence_df = pd.concat(sequence_df)

hexadecimal_alphabets = '0123456789ABCDEF'
color = ["#" + ''.join([random.choice(hexadecimal_alphabets) for j in range(6)]) for i in range(len(sequences))]

if len(sequence_df["parameters"].iloc[0])==1:
    plt.figure(figsize=(6, 6))
    if avg_1d == False:
        for count,seq in enumerate(sequences):
            seq_df = sequence_df.loc[sequence_df["seq"] == seq]
            seq_df =seq_df.sort_values(by=["parameters"])
            parameter_values = seq_df['parameters'].apply(lambda x: x[0])
            plt.plot(parameter_values, seq_df['translation'], color[count], linewidth=0.5, label=seq)
    else:
        all_parameters = []
        all_translations = []
        all_rotations = []
        # all_rates = []
        all_convergence_rates = []
        all_execution_rates = []

        for count,seq in enumerate(sequences):
            seq_df = sequence_df.loc[sequence_df["seq"] == seq]
            seq_df =seq_df.sort_values(by=["parameters"])
            parameter_values = seq_df['parameters'].apply(lambda x: x[0])

            # Extract parameter values and corresponding translations
            parameter_values = seq_df['parameters'].apply(lambda x: x[0]).tolist()
            translation_values = seq_df['translation'].tolist()
            rotation_values = seq_df['rotation'].tolist()
            # success_values = seq_df['success_rate'].tolist()
            convergence_values = seq_df['convergence_rates'].tolist()
            execution_values = seq_df['execution_rates'].tolist()

            
            # Append to lists
            all_parameters.append(parameter_values)
            all_translations.append(translation_values)
            all_rotations.append(rotation_values)
            # all_rates.append(success_values)
            all_convergence_rates.append(convergence_values)
            all_execution_rates.append(execution_values)

            
        # Get all unique parameters and sort them
        unique_parameters = sorted(set(p for param_list in all_parameters for p in param_list))

        # Prepare to calculate average translation values
        avg_translations = {param: [] for param in unique_parameters}
        avg_rotations = {param: [] for param in unique_parameters}
        # avg_rates = {param: [] for param in unique_parameters}
        avg_convergence_rates = {param: [] for param in unique_parameters}
        avg_execution_rates = {param: [] for param in unique_parameters}


        # Calculate the average translation values for each unique parameter
        for param in unique_parameters:
            param_translations = []
            param_rotations = []
            # param_rates = []
            param_convergence_rates = []
            param_execution_rates = []


            for i in range(len(all_parameters)):
                if param in all_parameters[i]:
                    index = all_parameters[i].index(param)
                    param_translations.append(all_translations[i][index])
                    param_rotations.append(all_rotations[i][index])
                    param_convergence_rates.append(all_convergence_rates[i][index])
                    param_execution_rates.append(all_execution_rates[i][index])
                    # if all_translations[i][index] == 0.0:
                    #     param_rates.append(0.0)
                    # else:
                    #     param_rates.append(all_rates[i][index])
                    
            if param_translations:
                if 0.0 in param_translations: 
                    avg_translations[param] = -1.0
                    avg_rotations[param] = -1.0

                else:
                    avg_translations[param] = sum(param_translations) / len(param_translations)
                    avg_rotations[param] = sum(param_rotations) / len(param_rotations)
            
            # avg_rates[param] = sum(param_rates) / len(param_rates)
            avg_convergence_rates[param] = sum(param_convergence_rates) / len(param_convergence_rates)
            avg_execution_rates[param] = sum(param_execution_rates) / len(param_execution_rates)

            

        # Convert the average translations to a DataFrame
        average_trans_df = pd.DataFrame(list(avg_translations.items()), columns=['parameters', 'average_translation'])
        average_trans_df = average_trans_df.sort_values(by=['parameters'])

        average_rotation_df = pd.DataFrame(list(avg_rotations.items()), columns=['parameters', 'average_rotation'])
        average_rotation_df = average_rotation_df.sort_values(by=['parameters'])

        avg_execution_rates_df = pd.DataFrame(list(avg_execution_rates.items()), columns=['parameters', 'average_execution_rates'])
        avg_execution_rates_df = avg_execution_rates_df.sort_values(by=['parameters'])

        avg_convergence_rates_df = pd.DataFrame(list(avg_convergence_rates.items()), columns=['parameters', 'average_convergence_rates'])
        avg_convergence_rates_df = avg_convergence_rates_df.sort_values(by=['parameters'])
        

        # min_trans = average_trans_df.sort_values(by=['average_translation'])
        average_rotation_df = average_rotation_df[average_rotation_df['average_rotation'] > 0]
        average_trans_df = average_trans_df[average_trans_df['average_translation'] > 0]

        min_row = average_trans_df.loc[average_trans_df['average_translation'].idxmin()]

        # Get the minimum value and associated 'param' value
        min_value = min_row['average_translation']
        param_value = min_row['parameters']
        min_rot = average_rotation_df.loc[average_rotation_df['parameters'] == param_value].values[0][1]
        
        print("Min average: "+str(min_value)+" % "+str(min_rot))
        print("At parameters: : "+str(param_value))

        # Plot the averaged graph
        plt.plot(average_trans_df['parameters'], average_trans_df['average_translation'], color='blue', linewidth=2, label='Averaged Error')
        
    plt.xlabel('Parameter')
    plt.ylabel('Translational Error(%)')
    plt.ylim((0, 5))
    plt.legend(loc="upper right")
    if graph_it: plt.savefig(os.path.join(path_to_save, config +"_translation"+ '.pdf'), pad_inches=0, bbox_inches='tight')
    plt.close()


    plt.figure(figsize=(6, 6))
    if avg_1d == False:
        for count,seq in enumerate(sequences):
            seq_df = sequence_df.loc[sequence_df["seq"] == seq]
            seq_df =seq_df.sort_values(by=["parameters"])
            parameter_values = seq_df['parameters'].apply(lambda x: x[0])
            plt.plot(parameter_values, seq_df['rotation'], color[count], linewidth=0.5, label=seq)
    else:
        plt.plot(average_rotation_df['parameters'], average_rotation_df['average_rotation'], color='red', linewidth=2, label='Averaged Error')


    plt.xlabel('Parameter')
    plt.ylabel('Rotational Error(deg/m)')
    plt.ylim((0, 0.05))
    plt.legend(loc="upper right")
    if graph_it: plt.savefig(os.path.join(path_to_save, config+"_rotation" + '.pdf'), pad_inches=0, bbox_inches='tight')
    plt.close()

    plt.figure(figsize=(6, 6))
    # if avg_1d == False:
    #     for count,seq in enumerate(sequences):
    #         seq_df = sequence_df.loc[sequence_df["seq"] == seq]
    #         seq_df =seq_df.sort_values(by=["parameters"])
    #         parameter_values = seq_df['parameters'].apply(lambda x: x[0])
    #         plt.plot(parameter_values, seq_df['convergence_rates'], color[count], linewidth=0.5, label=seq)
    #         plt.plot(parameter_values, seq_df['convergence_rates'], color[count], linewidth=0.5, label=seq)
    # else:
    #     # plt.plot(average_rates_df['parameters'], average_rates_df['average_rates'], color='green', linewidth=2, label='Averaged Error')
    plt.plot(avg_execution_rates_df['parameters'], avg_execution_rates_df['average_execution_rates'], color='red', linewidth=2, label='Average Completed Frames(%)')
    plt.plot(avg_convergence_rates_df['parameters'], avg_convergence_rates_df['average_convergence_rates'], color='blue', linewidth=2, label='Average Converged Frames(%)')



    plt.xlabel('Parameter')
    plt.ylabel('ICP Success Rate (%)')
    plt.ylim((80, 110))
    plt.legend(loc="lower right")
    if graph_it: plt.savefig(os.path.join(path_to_save, config+"_success_rates" + '.pdf'), pad_inches=0, bbox_inches='tight')
    plt.close()

elif len(sequence_df["parameters"].iloc[0])==2 and graph_mode == "3d":
    pairs = []
    x_points = []
    y_points = []
    avg_translation = []
    avg_rotation = []
    avg_rmse = []
    # avg_success_rate = []
    avg_convergence_rate = []
    avg_execution_rates = []
    success_rate_x = []
    success_rate_y = []

    min_avg = 100000
    min_pair = []
    min_rmse_avg = 100000
    min_rmse_pair = []

    # # success_rates=[]
    # convergence_rates=[]
    # execution_rates=[]

    # for index,row in sequence_df.iterrows():
    #     # rate = get_icp_failure_count(row['file'], row['seq'])
    #     percent_converged, percent_executed = get_icp_failure_count(row['file'], row['seq'])
    #     # success_rates.append(rate)
    #     convergence_rates.append(percent_converged)
    #     execution_rates.append(percent_executed)

    # # Add the success_rates list as a new column to the DataFrame
    # # sequence_df['success_rate'] = success_rates
    # sequence_df['convergence_rates'] = convergence_rates
    # sequence_df['execution_rates'] = execution_rates

    for count, param_pair in enumerate(sequence_df["parameters"]):
        if param_pair in pairs:continue

        filtered_df = sequence_df[sequence_df['parameters'].apply(lambda x: x == param_pair)]
        
        average_translation = filtered_df['translation'].mean()
        average_rotation = filtered_df['rotation'].mean()
        rmse = filtered_df['rmse'].mean()
        # rmse = filtered_df['rmse_99f9'].mean()

        # average_rate = filtered_df['success_rate'].mean()
        average_convergence_rate = filtered_df['convergence_rates'].mean()
        average_execution_rates = filtered_df['execution_rates'].mean()

        avg_convergence_rate.append(average_convergence_rate)
        avg_execution_rates.append(average_execution_rates)
        # avg_success_rate.append(average_rate)
        success_rate_x.append(param_pair[0])
        success_rate_y.append(param_pair[1])

        # print(average_translation, " ", param_pair)
        pairs.append(param_pair)

        if (filtered_df['translation'] == 0.0).any().any() or (filtered_df['rotation'] == 0.0).any().any(): continue
        if len(filtered_df['translation']) < len(sequences): continue
        # if len(filtered_df['rotation']) < len(sequences): continue

        if average_translation == 0.0 or average_rotation ==0.0: continue
        if average_translation < min_avg:
            min_avg = average_translation
            min_pair = param_pair
            min_rot = average_rotation

        if rmse < min_rmse_avg:
            min_rmse_avg = rmse
            min_rmse_pair = param_pair

        x_points.append(param_pair[0])
        y_points.append(param_pair[1])
        avg_translation.append(average_translation)
        avg_rotation.append(average_rotation)
        avg_rmse.append(rmse)

    print("Min average: "+str(min_avg)+" % "+str(min_rot))
    print("At parameters: : "+str(min_pair))

    print("Min rmse average: "+str(min_rmse_avg))
    print("At parameters: : "+str(min_rmse_pair))


    x_points = np.array(x_points)
    y_points = np.array(y_points)
    avg_translation = np.array(avg_translation)
    avg_rotation = np.array(avg_rotation)
    avg_rmse = np.array(avg_rmse)
    # avg_success_rate = np.array(avg_success_rate)
    avg_convergence_rate = np.array(avg_convergence_rate)
    avg_execution_rates = np.array(avg_execution_rates)
    success_rate_x = np.array(success_rate_x)
    success_rate_y = np.array(success_rate_y)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_trisurf(x_points, y_points, avg_translation, cmap='magma', edgecolor='none',alpha=0.8)
    fig.colorbar(surf, shrink=0.5, aspect=5)

    ax.set_xlim(x_points.min(), x_points.max())
    ax.set_ylim(y_points.min(), y_points.max())
    ax.set_zlim(0, 5)
    ax.set_xlabel('Parameter 1')
    ax.set_ylabel('Parameter 2')
    ax.set_zlabel('Translational Error(%)')
    ax.set_title('Parameter Search for '+config)

    ax.view_init(30, 45)
    if graph_it: plt.savefig(os.path.join(path_to_save, config +"_translation_3d"+ '.pdf'), pad_inches=0, bbox_inches='tight')

    ax.view_init(90, 90)
    if graph_it: plt.savefig(os.path.join(path_to_save, config +"_translation_3d_birdseye"+ '.pdf'), pad_inches=0, bbox_inches='tight')

    ax.view_init(0, 90)
    if graph_it: plt.savefig(os.path.join(path_to_save, config +"_translation_3d_side1"+ '.pdf'), pad_inches=0, bbox_inches='tight')
    ax.view_init(0, 0)
    if graph_it: plt.savefig(os.path.join(path_to_save, config +"_translation_3d_side2"+ '.pdf'), pad_inches=0, bbox_inches='tight')
    plt.close()

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_trisurf(x_points, y_points, avg_rotation, cmap='magma', edgecolor='none',alpha=0.8)
    fig.colorbar(surf, shrink=0.5, aspect=5)

    ax.set_xlim(x_points.min(), x_points.max())
    ax.set_ylim(y_points.min(), y_points.max())
    ax.set_zlim(0, 0.015)
    ax.set_xlabel('Parameter 1')
    ax.set_ylabel('Parameter 2')
    ax.set_zlabel('Rotational Error(deg/m)')
    ax.set_title('Parameter Search for '+config)
    ax.view_init(30, 45)

    if graph_it: plt.savefig(os.path.join(path_to_save, config +"_rotational_3d"+ '.pdf'), pad_inches=0, bbox_inches='tight')
    plt.close()


    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_trisurf(x_points, y_points, avg_rmse, cmap='magma', edgecolor='none',alpha=0.8)
    fig.colorbar(surf, shrink=0.5, aspect=5)

    ax.set_xlim(x_points.min(), x_points.max())
    ax.set_ylim(y_points.min(), y_points.max())
    ax.set_zlim(0.0, 40)
    ax.set_xlabel('Parameter 1')
    ax.set_ylabel('Parameter 2')
    ax.set_zlabel('RMSE (m)')
    ax.set_title('Parameter Search for '+config)
    ax.view_init(30, 45)

    if graph_it: plt.savefig(os.path.join(path_to_save, config +"_rmse_3d"+ '.pdf'), pad_inches=0, bbox_inches='tight')

    ax.view_init(90, 90)
    if graph_it: plt.savefig(os.path.join(path_to_save, config +"_rmse_3d_birdseye"+ '.pdf'), pad_inches=0, bbox_inches='tight')

    ax.view_init(0, 90)
    if graph_it: plt.savefig(os.path.join(path_to_save, config +"_rmse_3d_side1"+ '.pdf'), pad_inches=0, bbox_inches='tight')
    ax.view_init(0, 0)
    if graph_it: plt.savefig(os.path.join(path_to_save, config +"_rmse_3d_side2"+ '.pdf'), pad_inches=0, bbox_inches='tight')
    plt.close()
    plt.close()


    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_trisurf(success_rate_x, success_rate_y, avg_convergence_rate, cmap='magma', edgecolor='none',alpha=0.8)
    fig.colorbar(surf, shrink=0.5, aspect=5)

    ax.set_xlim(success_rate_x.min(), success_rate_x.max())
    ax.set_ylim(success_rate_y.min(), success_rate_y.max())
    ax.set_zlim(80.0, avg_convergence_rate.max())
    ax.set_xlabel('Parameter 1')
    ax.set_ylabel('Parameter 2')
    ax.set_zlabel('ICP Average Converged Frames(%)')
    ax.set_title('ICP Average Convergence Rate (%)')
    # ax.view_init(30, 45)
    # if graph_it: plt.savefig(os.path.join(path_to_save, config +"_rmse_3d"+ '.pdf'), pad_inches=0, bbox_inches='tight')

    ax.view_init(90, 90)
    if graph_it: 
        plt.savefig(os.path.join(path_to_save, config +"_convergence_rates_birdseye"+ '.pdf'), pad_inches=0, bbox_inches='tight')

    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_trisurf(success_rate_x, success_rate_y, avg_execution_rates, cmap='magma', edgecolor='none',alpha=0.8)
    fig.colorbar(surf, shrink=0.5, aspect=5)

    ax.set_xlim(success_rate_x.min(), success_rate_x.max())
    ax.set_ylim(success_rate_y.min(), success_rate_y.max())
    ax.set_zlim(80.0, avg_execution_rates.max())
    ax.set_xlabel('Parameter 1')
    ax.set_ylabel('Parameter 2')
    ax.set_zlabel('ICP Average Completed Frames(%)')
    ax.set_title('ICP Average Execution Rate (%)')

    ax.view_init(90, 90)
    if graph_it: 
        plt.savefig(os.path.join(path_to_save, config +"_execution_rates_birdseye"+ '.pdf'), pad_inches=0, bbox_inches='tight')


elif len(sequence_df["parameters"].iloc[0])==2 and graph_mode == "2d":
    pairs = []
    x_points = []
    y_points = []
    avg_translation = []
    avg_rotation = []
    avg_rmse = []

    min_avg = 100000
    min_pair = []

    color = ["#" + ''.join([random.choice(hexadecimal_alphabets) for j in range(6)]) for i in range(len(sequences)*3)]
    seq_param_pair= []
    plt.figure(figsize=(6, 6))

    #INDIVIDUAL SEQUENCES 
    # for count,seq in enumerate(sequences):
    #     seq_df = sequence_df.loc[sequence_df["seq"] == seq]

    #     # x = seq_df['parameters'].apply(lambda x: x[0])
        # y = seq_df['parameters'].apply(lambda x: x[1])
        # # z = seq_df['translation']

        # for count_2,param_2 in enumerate(y):
        #     if param_2 != 0.2 and param_2 != 0.19999999999999998:continue
        #     if [seq,param_2] in seq_param_pair:continue

    #         # seq_df_2 = seq_df[seq_df["parameters"].apply(lambda x: x[1] == param_2)]
    #         seq_df_2 = seq_df[seq_df["parameters"].apply(lambda x: x[1] == 0.2 or x[1] == 0.19999999999999998)]
    #         seq_df_2 =seq_df_2.sort_values(by=["parameters"][0])

    #         x = seq_df_2['parameters'].apply(lambda x: x[0])
            # plt.plot(x, seq_df_2['translation'], color[count*3], linewidth=0.5, label=seq)

    #         # plt.plot(x, seq_df_2['translation'], color[count_2], linewidth=0.5)
    #         seq_param_pair.append([seq,param_2])

    #AVERAGE SEQUENCE
    for count, param_pair in enumerate(sequence_df["parameters"]):
        if param_pair in pairs:continue

        if param_pair[1] != 0.2 and param_pair[1] != 0.19999999999999998:continue
        # if param_pair[0] != 25:continue


        filtered_df = sequence_df[sequence_df['parameters'].apply(lambda x: x == param_pair)]
        
        average_translation = filtered_df['translation'].mean()
        average_rotation = filtered_df['rotation'].mean()
        rmse = filtered_df['rmse'].mean()

        pairs.append(param_pair)

        if (filtered_df['translation'] == 0.0).any().any() or (filtered_df['rotation'] == 0.0).any().any(): continue
        if len(filtered_df['translation']) < len(sequences): continue
        if len(filtered_df['rotation']) < len(sequences): continue

        if average_translation == 0.0 or average_rotation ==0.0: continue
        if average_translation < min_avg:
            min_avg = average_translation
            min_pair = param_pair

        x_points.append(param_pair[0])
        # y_points.append(param_pair[1])
        avg_translation.append(average_translation)
        # avg_rotation.append(average_rotation)
        # avg_rmse.append(rmse)
    print(x_points)
    print(avg_translation)
    # avg_huber1=[1.9279028730888947, 2.355444016088601, 2.659048517609654, 2.123187067769668, 1.6221646137431802, 2.1530711070329644]
    # avg_huber2=[3.8131487826311092, 4.54622494422521, 5.145264337991337, 4.223014857865033, 2.7000934185449594, 2.6079393629266234]
    # avg_huber3=[5.882643957284889, 6.4111342075778435, 7.145014445160461, 5.7049760255110185, 3.7742892042612035, 3.1416888759011647]
    # avg_huber1 =[1.9097131738637514, 1.87578414138931, 1.6973971054303796, 1.7110187079759134, 1.6221646137431802, 1.744175208930757, 1.6936262867601524, 1.7535342358354242, 1.7996826944571698, 1.793897101063591, 1.8681154313832764, 1.9029868418813571, 2.096158040679658, 2.138370372680536]
    # avg_huber2 = [3.3257326043244984, 3.190403583305586, 2.973401590009214, 3.012427694608802, 2.7000934185449594, 2.7534551327183765, 2.7729862023477114, 2.6113430489777616, 2.50910692443526, 2.5454088316983596, 2.6695207382351676, 2.6394380342260884, 2.8662810775229417, 2.652311906693138]
    # avg_huber3 = [4.364492730819013, 4.218430757310904, 4.166366430091648, 4.1194453522492545, 3.7742892042612035, 3.838963133063957, 3.7737538528883703, 3.6892726267840557, 3.6035191079689, 3.4416694536653925, 3.3308765036563694, 3.6031193426836965, 3.579443520965719]
    # plt.plot(x_points, avg_translation, linewidth=0.5)
    avg_trim100_1 = [2.5727522284728463, 2.223888612212194, 2.31263819732823, 2.147113418818154, 2.016551159110031, 2.0295832990103, 1.986410601727516, 2.053184103266611, 2.3624232506023817, 2.1150143181816525, 2.126206262779609, 2.053116387821417, 2.362488752422631, 2.2228831117197347, 2.343972485218736, 2.3350280789481133]
    avg_trim100_2 = [1.8845847376662197, 1.751268694834034, 1.80937849061497, 1.7122523282047268, 1.8875105728646624, 1.7424055044649591, 2.0492210062394727, 1.94814211222936, 1.9026779019881797, 2.0101527518481173, 1.9327425987801188, 2.023744267881881, 1.9214154722724264, 2.2408290883426005, 2.1361438751301454, 2.175473092923537]
    avg_trim100_3 = [2.626108094947552, 2.6467543779940548, 2.4128995105807585, 2.7019376625646627, 2.396140393655936, 2.526200970656694, 2.2171517323800765, 2.367363931278523, 2.1787149758401987, 2.354380090220341, 2.319083075566694, 2.255908660616694, 2.121458308417012, 2.514537962701081, 2.3140105641744646, 2.1628849267059445]
    avg_trim100_4 = [2.0685132831311104, 1.985710960003474, 1.7365683010093078, 1.902438027390364, 2.0076449800137026, 1.925266791988259, 1.9296115571555743, 1.9858948577365985, 1.916358638844048, 2.2324367653035533, 1.9642726010249365, 2.047885254495817, 2.4226581888368863, 2.0047836987528367, 2.267218306280387, 2.215902711815987]
    plt.plot(x_points, avg_trim100_1, 'r', linewidth=0.5, label="seq1")
    plt.plot(x_points, avg_trim100_2, 'g', linewidth=0.5, label="seq2")
    plt.plot(x_points, avg_trim100_3, 'b', linewidth=0.5, label="seq3")
    plt.plot(x_points, avg_trim100_4, 'c', linewidth=0.5, label="seq4")

    plt.xlabel('Parameter')
    plt.ylabel('Translational Error(%)')
    plt.ylim((0, 8))
    plt.legend(loc="upper right")
    if graph_it: plt.savefig(os.path.join(path_to_save, config +"_translation_2d_0f2_trim"+ '.pdf'), pad_inches=0, bbox_inches='tight')
    plt.close()


#RMSE PLOTTING
    # seq_param_pair= []
    # plt.figure(figsize=(6, 6))
    # for count,seq in enumerate(sequences):
    #     seq_df = sequence_df.loc[sequence_df["seq"] == seq]
    #     y = seq_df['parameters'].apply(lambda x: x[1])
    #     for count_2,param_2 in enumerate(y):
    #         if param_2 != 0.2 and param_2 != 0.19999999999999998:continue
    #         if [seq,param_2] in seq_param_pair:continue

    #         seq_df_2 = seq_df[seq_df["parameters"].apply(lambda x: x[1] == 0.2 or x[1] == 0.19999999999999998)]
    #         seq_df_2 =seq_df_2.sort_values(by=["parameters"][0])

    #         x = seq_df_2['parameters'].apply(lambda x: x[0])
    #         plt.plot(x, seq_df_2['rmse'], color[count*3+1], linewidth=0.5, label=seq+"RMSE")
    #         plt.plot(x, seq_df_2['rmse_99f9'], color[count*3+2], linewidth=0.5, label=seq+"RMSE_99f9")
    #         seq_param_pair.append([seq,param_2])

    # plt.xlabel('Parameter')
    # plt.ylabel('Translational Relative Error RMSE (m)')
    # plt.ylim((0.03, 0.07))
    # plt.legend(loc="upper right")
    # if graph_it: plt.savefig(os.path.join(path_to_save, config +"_translation_rmse_2d_0f2"+ '.pdf'), pad_inches=0, bbox_inches='tight')
    # plt.close()


