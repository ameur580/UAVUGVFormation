# file to be used in the mission planning in the project 6/1/2024
# author Ameur

import yaml

def load_yaml(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def save_yaml(data, file_path):
    with open(file_path, 'w') as file:
        yaml.dump(data, file, default_flow_style=False, sort_keys=False)

def load_data():
    mission_file_path = 'mission.yaml'
    ugv_file_path = 'UGV.yaml'
    uav_file_path = 'UAV.yaml'

    mission_data = load_yaml(mission_file_path)
    ugv_data = load_yaml(ugv_file_path)['UGV']
    uav_data = load_yaml(uav_file_path)['UAV']
    return ugv_data, uav_data, mission_data

def filter_robots_for_task(robot_data, task, robot_type):
    filtered_robots = []
    matching_robots = []

    required_capabilities = set(task['capabilities'])
    required_robot_count = task[f'nb_{robot_type}']

    for robot in robot_data:
        robot_capabilities = set(robot['capabilities'])
        if required_capabilities.issubset(robot_capabilities):
            filtered_robots.append(robot)
    if len(filtered_robots) >= required_robot_count:
        matching_robots = filtered_robots[:required_robot_count]

    return matching_robots

def assign_subtasks_to_robots(robot_list, subtasks):
    for robot in robot_list:
        robot['subtasks'] = subtasks
    return robot_list

def missionProcess(ugv_data, uav_data, mission_data):
    filtered_ugvs = []
    filtered_uavs = []
    task_ugvs_with_subtasks = []
    task_uavs_with_subtasks = []
    for task in mission_data['mission']['tasks']:
        if task['status'] == 'not started':
            i = 0
            j = 0
            if task['robot_type'] == 'UGV':
                task_ugvs = filter_robots_for_task(ugv_data, task, 'UGV')
                for subtask in task['subtasks']:
                    task_ugvs_with_subtasks.append(subtask['name'])
                assign_subtasks_to_robots(task_ugvs, task_ugvs_with_subtasks)
                for ugv in task_ugvs:
                    ugv['initialPosition'] = mission_data['mission']['robotsInitialPositions'][i]
                    i = i+1
                filtered_ugvs.extend(task_ugvs)
            elif task['robot_type'] == 'UAV':
                task_uavs = filter_robots_for_task(uav_data, task, 'UAV')
                for subtask in task['subtasks']:
                    task_uavs_with_subtasks.append( subtask['name'])
                assign_subtasks_to_robots(task_uavs, task_uavs_with_subtasks)
                for uav in task_uavs:
                    uav['initialPosition'] = mission_data['mission']['robotsInitialPositions'][i]
                    i = i+1
                filtered_uavs.extend(task_uavs)
    if len(filtered_uavs) > 0:
        if len(filtered_ugvs) > 0:
            output_data = {'tasks with assigned UGVs': task_ugvs_with_subtasks,
                           'filtered_ugvs': filtered_ugvs, 
                           'tasks with assigned UAVs': task_uavs_with_subtasks,
                           'filtered_uavs': filtered_uavs}
        else:
            output_data = {'tasks with assigned UAVs': task_uavs_with_subtasks,'filtered_uavs': filtered_uavs}
    elif len(filtered_ugvs) > 0:
        output_data = {'tasks with assigned UGVs': task_ugvs_with_subtasks, 'filtered_ugvs': filtered_ugvs}
    return output_data


def main():

    ugv_data, uav_data, mission_data = load_data()
    output_file_path = 'filtered_robots.yaml'

    output_data = missionProcess(ugv_data, uav_data, mission_data)

#    print(f"output_data==> {output_data}")

    save_yaml(output_data, output_file_path)
    print(f"Filtered robots saved to {output_file_path}")

if __name__ == "__main__":
    main()
