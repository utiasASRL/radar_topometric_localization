import matplotlib.pyplot as plt
from matplotlib import cm
import os
import fnmatch
import pandas as pd
import random
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import re

# USER INPUT: Set True to generate graph pdfs
graph_it = True

# USER INPUT: Set True to average sequences of single parameter sweeps
avg_1d = True

# USER INPUT: Set the extractor ("kstrongest", "cacfar", "caso_cfar", "cago_cfar", "bfar", "oscfar", "tm_cfar", "msca_cfar", "is_cfar", "vi_cfar", "cen2018", "cen2019", "cfear_kstrong")
config = "kstrongest"

# USER INPUT: Set the sequences to be averaged and graphed
sequences = ['boreas-2021-11-14-09-47', 'boreas-2021-11-16-14-10', 'boreas-2021-11-23-14-27']

ROOTDIR = os.environ['ROOTDIR']
VTRRESULT = os.environ['VTRRESULT']
VTRRROOT = os.environ['VTRRROOT']
VTRRDATA = os.environ['VTRRDATA']
SENSOR = "radar"

# Set results subfolder, VTRRESULT is set in setup_container.sh
VTRRRESULT = os.path.join(VTRRESULT,SENSOR)
os.environ['VTRRRESULT'] = VTRRRESULT

# Returns the extractor parameters from a particular sequence given the folder name
# Ex boreas-2021-11-28-09-18_kstrongest_3_0f35 -> [3, 0.35]
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

# Returns the sequence name from a particular folder name
# Ex boreas-2021-11-28-09-18_kstrongest_3_0f35 -> boreas-2021-11-28-09-18
def parse_seq_from_name(name):
    return name.split("_")[0]

# Parse the log file of a particular sequence. Later used to get the number of total radar frames
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
    
# Function to get the number of ICP failures from a specific sequence with its respective a log file
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

    contents = read_first_lines(most_recent_file, num_lines=100)

    # Use regular expression to find the number in the format "Found X radar data"
    match = re.search(r"Found (\d+) radar data", contents)
    
    frame_count = 0
    # Extract and return the number if found
    if match:
        frame_count = int(match.group(1))-1
    else:
        raise ValueError("The pattern 'Found X radar data' was not found in the first lines of the log file.")

    ICP_fail_count = 0
    last_frame_executed = 0

    # Parse the log file to get the number of ICP failures
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

def main():
    sequence_df = []

    for count, seq in enumerate(sequences):
        path_to_save = os.path.join(VTRRRESULT, "detectors", config, "parameter_graphs")

        if not os.path.exists(path_to_save):
            print("Creating directory: " + path_to_save)
            os.makedirs(path_to_save)

        csv_prefix = seq + "_" + config

        matching_files = []

        for file in os.listdir(os.path.join(VTRRRESULT, "detectors", config, "data", seq)):
            if fnmatch.fnmatch(file, f'*{csv_prefix}*.csv'):
                matching_files.append(os.path.join(VTRRRESULT, "detectors", config, "data", seq, file))

        # Read csv files and convert to pandas table
        dfs = []
        for file_path in matching_files:
            if file_path.endswith(".csv"):
                df = pd.read_csv(file_path, sep=' ', names=["file", "translation", "rotation", "convergence_rates", "execution_rates"])
                
                # Check if the ICP convergence and execution rates have been calculated already
                if df['convergence_rates'].isna().any() or df['execution_rates'].isna().any():
                    seq_name = file_path.split("/")[-2]
                    for index, row in df.iterrows():
                        filename = row["file"]
                        percent_converged, percent_executed = get_icp_failure_count(filename, seq_name)
                        df.at[index, 'convergence_rates'] = percent_converged
                        df.at[index, 'execution_rates'] = percent_executed
                    df.to_csv(file_path, sep=' ', header=False, index=False)

                dfs.append(df)

        combined_df = pd.concat(dfs, ignore_index=True)

        combined_df['parameters'] = combined_df['file'].apply(parse_number_from_name)
        combined_df['seq'] = combined_df['file'].apply(parse_seq_from_name)

        sequence_df.append(combined_df)

    sequence_df = pd.concat(sequence_df)

    # Create list of random colours for the graphs
    hexadecimal_alphabets = '0123456789ABCDEF'
    color = ["#" + ''.join([random.choice(hexadecimal_alphabets) for j in range(6)]) for i in range(len(sequences))]

    if len(sequence_df["parameters"].iloc[0]) == 1:
        plt.figure(figsize=(6, 6))
        if avg_1d == False:
            for count, seq in enumerate(sequences):
                seq_df = sequence_df.loc[sequence_df["seq"] == seq]
                seq_df = seq_df.sort_values(by=["parameters"])
                parameter_values = seq_df['parameters'].apply(lambda x: x[0])
                plt.plot(parameter_values, seq_df['translation'], color[count], linewidth=0.5, label=seq)
        else:
            all_parameters = []
            all_translations = []
            all_rotations = []
            all_convergence_rates = []
            all_execution_rates = []

            for count, seq in enumerate(sequences):
                seq_df = sequence_df.loc[sequence_df["seq"] == seq]
                seq_df = seq_df.sort_values(by=["parameters"])
                parameter_values = seq_df['parameters'].apply(lambda x: x[0])

                # Extract parameter values and corresponding translations
                parameter_values = seq_df['parameters'].apply(lambda x: x[0]).tolist()
                translation_values = seq_df['translation'].tolist()
                rotation_values = seq_df['rotation'].tolist()
                convergence_values = seq_df['convergence_rates'].tolist()
                execution_values = seq_df['execution_rates'].tolist()

                # Append to lists
                all_parameters.append(parameter_values)
                all_translations.append(translation_values)
                all_rotations.append(rotation_values)
                all_convergence_rates.append(convergence_values)
                all_execution_rates.append(execution_values)

            # Get all unique parameters and sort them
            unique_parameters = sorted(set(p for param_list in all_parameters for p in param_list))

            # Prepare to calculate average translation values
            avg_translations = {param: [] for param in unique_parameters}
            avg_rotations = {param: [] for param in unique_parameters}
            avg_convergence_rates = {param: [] for param in unique_parameters}
            avg_execution_rates = {param: [] for param in unique_parameters}

            # Calculate the average translation values for each unique parameter
            for param in unique_parameters:
                param_translations = []
                param_rotations = []
                param_convergence_rates = []
                param_execution_rates = []

                for i in range(len(all_parameters)):
                    if param in all_parameters[i]:
                        index = all_parameters[i].index(param)
                        param_translations.append(all_translations[i][index])
                        param_rotations.append(all_rotations[i][index])
                        param_convergence_rates.append(all_convergence_rates[i][index])
                        param_execution_rates.append(all_execution_rates[i][index])

                if param_translations:
                    if 0.0 in param_translations:
                        avg_translations[param] = -1.0
                        avg_rotations[param] = -1.0
                    else:
                        avg_translations[param] = sum(param_translations) / len(param_translations)
                        avg_rotations[param] = sum(param_rotations) / len(param_rotations)

                avg_convergence_rates[param] = sum(param_convergence_rates) / len(param_convergence_rates)
                avg_execution_rates[param] = sum(param_execution_rates) / len(param_execution_rates)

            # Convert the average translations to a DataFrame
            average_translation_df = pd.DataFrame(list(avg_translations.items()), columns=['parameters', 'average_translation'])
            average_translation_df = average_translation_df.sort_values(by=['parameters'])

            average_rotation_df = pd.DataFrame(list(avg_rotations.items()), columns=['parameters', 'average_rotation'])
            average_rotation_df = average_rotation_df.sort_values(by=['parameters'])

            avg_execution_rates_df = pd.DataFrame(list(avg_execution_rates.items()), columns=['parameters', 'average_execution_rates'])
            avg_execution_rates_df = avg_execution_rates_df.sort_values(by=['parameters'])

            avg_convergence_rates_df = pd.DataFrame(list(avg_convergence_rates.items()), columns=['parameters', 'average_convergence_rates'])
            avg_convergence_rates_df = avg_convergence_rates_df.sort_values(by=['parameters'])

            average_rotation_df = average_rotation_df[average_rotation_df['average_rotation'] > 0]
            average_translation_df = average_translation_df[average_translation_df['average_translation'] > 0]

            min_row = average_translation_df.loc[average_translation_df['average_translation'].idxmin()]

            # Get the minimum value and associated 'param' value
            min_value = min_row['average_translation']
            param_value = min_row['parameters']
            min_rot = average_rotation_df.loc[average_rotation_df['parameters'] == param_value].values[0][1]

            print("Min average: " + str(min_value) + " % " + str(min_rot))
            print("At parameters: : " + str(param_value))

            # Plot the averaged graph
            plt.plot(average_translation_df['parameters'], average_translation_df['average_translation'], color='blue', linewidth=2, label='Averaged Error')

        plt.xlabel('Parameter')
        plt.ylabel('Translational Error(%)')
        plt.ylim((0, 5))
        plt.legend(loc="upper right")
        if graph_it:
            plt.savefig(os.path.join(path_to_save, config + "_translation" + '.pdf'), pad_inches=0, bbox_inches='tight')
        plt.close()

        plt.figure(figsize=(6, 6))
        if avg_1d == False:
            for count, seq in enumerate(sequences):
                seq_df = sequence_df.loc[sequence_df["seq"] == seq]
                seq_df = seq_df.sort_values(by=["parameters"])
                parameter_values = seq_df['parameters'].apply(lambda x: x[0])
                plt.plot(parameter_values, seq_df['rotation'], color[count], linewidth=0.5, label=seq)
        else:
            plt.plot(average_rotation_df['parameters'], average_rotation_df['average_rotation'], color='red', linewidth=2, label='Averaged Error')

        plt.xlabel('Parameter')
        plt.ylabel('Rotational Error(deg/m)')
        plt.ylim((0, 0.05))
        plt.legend(loc="upper right")
        if graph_it:
            plt.savefig(os.path.join(path_to_save, config + "_rotation" + '.pdf'), pad_inches=0, bbox_inches='tight')
        plt.close()

        plt.figure(figsize=(6, 6))
        plt.plot(avg_execution_rates_df['parameters'], avg_execution_rates_df['average_execution_rates'], color='red', linewidth=2, label='Average Completed Frames(%)')
        plt.plot(avg_convergence_rates_df['parameters'], avg_convergence_rates_df['average_convergence_rates'], color='blue', linewidth=2, label='Average Converged Frames(%)')

        plt.xlabel('Parameter')
        plt.ylabel('ICP Success Rate (%)')
        plt.ylim((80, 110))
        plt.legend(loc="lower right")
        if graph_it:
            plt.savefig(os.path.join(path_to_save, config + "_success_rates" + '.pdf'), pad_inches=0, bbox_inches='tight')
        plt.close()

    elif len(sequence_df["parameters"].iloc[0]) == 2:
        pairs = []
        x_points = []
        y_points = []
        avg_translation = []
        avg_rotation = []
        avg_convergence_rate = []
        avg_execution_rates = []
        success_rate_x = []
        success_rate_y = []

        min_avg = 100000
        min_pair = []

        for count, param_pair in enumerate(sequence_df["parameters"]):
            if param_pair in pairs:
                continue

            filtered_df = sequence_df[sequence_df['parameters'].apply(lambda x: x == param_pair)]

            average_translation = filtered_df['translation'].mean()
            average_rotation = filtered_df['rotation'].mean()

            average_convergence_rate = filtered_df['convergence_rates'].mean()
            average_execution_rates = filtered_df['execution_rates'].mean()

            avg_convergence_rate.append(average_convergence_rate)
            avg_execution_rates.append(average_execution_rates)
            success_rate_x.append(param_pair[0])
            success_rate_y.append(param_pair[1])

            pairs.append(param_pair)

            if (filtered_df['translation'] == 0.0).any().any() or (filtered_df['rotation'] == 0.0).any().any():
                continue
            if len(filtered_df['translation']) < len(sequences):
                continue

            if average_translation == 0.0 or average_rotation == 0.0:
                continue
            if average_translation < min_avg:
                min_avg = average_translation
                min_pair = param_pair
                min_rot = average_rotation

            x_points.append(param_pair[0])
            y_points.append(param_pair[1])
            avg_translation.append(average_translation)
            avg_rotation.append(average_rotation)

        print("Min average: " + str(min_avg) + " % " + str(min_rot))
        print("At parameters: : " + str(min_pair))

        x_points = np.array(x_points)
        y_points = np.array(y_points)
        avg_translation = np.array(avg_translation)
        avg_rotation = np.array(avg_rotation)
        avg_convergence_rate = np.array(avg_convergence_rate)
        avg_execution_rates = np.array(avg_execution_rates)
        success_rate_x = np.array(success_rate_x)
        success_rate_y = np.array(success_rate_y)

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        surf = ax.plot_trisurf(x_points, y_points, avg_translation, cmap='magma', edgecolor='none', alpha=0.8)
        fig.colorbar(surf, shrink=0.5, aspect=5)

        ax.set_xlim(x_points.min(), x_points.max())
        ax.set_ylim(y_points.min(), y_points.max())
        ax.set_zlim(0, 5)
        ax.set_xlabel('Parameter 1')
        ax.set_ylabel('Parameter 2')
        ax.set_zlabel('Translational Error(%)')
        ax.set_title('Parameter Search for ' + config)

        ax.view_init(30, 45)
        if graph_it:
            plt.savefig(os.path.join(path_to_save, config + "_translation_3d" + '.pdf'), pad_inches=0, bbox_inches='tight')

        ax.view_init(90, 90)
        if graph_it:
            plt.savefig(os.path.join(path_to_save, config + "_translation_3d_birdseye" + '.pdf'), pad_inches=0, bbox_inches='tight')

        ax.view_init(0, 90)
        if graph_it:
            plt.savefig(os.path.join(path_to_save, config + "_translation_3d_side1" + '.pdf'), pad_inches=0, bbox_inches='tight')
        ax.view_init(0, 0)
        if graph_it:
            plt.savefig(os.path.join(path_to_save, config + "_translation_3d_side2" + '.pdf'), pad_inches=0, bbox_inches='tight')
        plt.close()

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        surf = ax.plot_trisurf(x_points, y_points, avg_rotation, cmap='magma', edgecolor='none', alpha=0.8)
        fig.colorbar(surf, shrink=0.5, aspect=5)

        ax.set_xlim(x_points.min(), x_points.max())
        ax.set_ylim(y_points.min(), y_points.max())
        ax.set_zlim(0, 0.015)
        ax.set_xlabel('Parameter 1')
        ax.set_ylabel('Parameter 2')
        ax.set_zlabel('Rotational Error(deg/m)')
        ax.set_title('Parameter Search for ' + config)
        ax.view_init(30, 45)

        if graph_it:
            plt.savefig(os.path.join(path_to_save, config + "_rotational_3d" + '.pdf'), pad_inches=0, bbox_inches='tight')
        plt.close()

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        surf = ax.plot_trisurf(success_rate_x, success_rate_y, avg_convergence_rate, cmap='magma', edgecolor='none', alpha=0.8)
        fig.colorbar(surf, shrink=0.5, aspect=5)

        ax.set_xlim(success_rate_x.min(), success_rate_x.max())
        ax.set_ylim(success_rate_y.min(), success_rate_y.max())
        ax.set_zlim(80.0, avg_convergence_rate.max())
        ax.set_xlabel('Parameter 1')
        ax.set_ylabel('Parameter 2')
        ax.set_zlabel('ICP Average Converged Frames(%)')
        ax.set_title('ICP Average Convergence Rate (%)')

        ax.view_init(90, 90)
        if graph_it:
            plt.savefig(os.path.join(path_to_save, config + "_convergence_rates_birdseye" + '.pdf'), pad_inches=0, bbox_inches='tight')

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        surf = ax.plot_trisurf(success_rate_x, success_rate_y, avg_execution_rates, cmap='magma', edgecolor='none', alpha=0.8)
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
            plt.savefig(os.path.join(path_to_save, config + "_execution_rates_birdseye" + '.pdf'), pad_inches=0, bbox_inches='tight')

if __name__ == "__main__":
        main()