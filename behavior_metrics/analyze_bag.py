import os
import json
import yaml
# import rosbag
import sys
import argparse
import cv2
from cv_bridge import CvBridge
import numpy as np
import pickle
import matplotlib.pyplot as plt

ros_version = os.environ.get('ROS_VERSION',"2")

if ros_version == "2":
    import rclpy
    try:
        import rosbag2_py
        from rosbag2_py import StorageOptions, ConverterOptions, SequentialReader
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        print("Error: rosbag2_py module not found")
        # from rclpy.node import Node
else:
    import rospy
    import rosbag
    
topic_type_map = {
  '/carla/ego_vehicle/odometry': 'nav_msgs/msg/Odometry',
  '/carla/ego_vehicle/collision': 'carla_msgs/msg/CarlaCollisionEvent',
  '/carla/ego_vehicle/lane_invasion': 'carla_msgs/msg/CarlaLaneInvasionEvent',
  '/carla/ego_vehicle/speedometer': 'std_msgs/msg/Float32',
  '/carla/ego_vehicle/vehicle_status': 'carla_msgs/msg/CarlaEgoVehicleStatus',
  '/carla/ego_vehicle/rgb_front/image': 'sensor_msgs/msg/Image',
  '/clock': 'rosgraph_msgs/msg/Clock',
  '/metadata': 'std_msgs/msg/String',             
  '/experiment_metrics': 'std_msgs/msg/String', 
  '/first_image': 'sensor_msgs/msg/Image'
  
}
    
def list_topics_ros1(bag_file):
    bag = rosbag.Bag(bag_file)
    topics = set(t for t,_,_ in bag.read_messages())
    bag.close()
    return topics

def list_topics_ros2(bag_dir):
    storage_opts = StorageOptions(uri=bag_dir, storage_id='sqlite3')
    conv_opts = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    reader = SequentialReader()
    reader.open(storage_opts, conv_opts)
    topics = {meta.name for meta in reader.get_all_topics_and_types()}
    return topics

def process_bag_msgs_ros1(bag_msgs, all_data):
    """
    Processes massages from a ROS1 bag (F1 simulation) and updates all_data.
    - bag_msgs: list of tuples (topic, msg, timestamp)
    - all_data: dglobal dictionary to update
    """
    import yaml, json
    from cv_bridge import CvBridge
    import numpy as np

    x_points, y_points = [], []
    metadata = {}
    experiment_metrics = {}
    first_image = np.zeros((1,1))
    bridge = CvBridge()

    # Extraction of data from the bag
    for topic, msg, t in bag_msgs:
        if topic == '/F1ROS/odom':
            d = yaml.load(str(msg), Loader=yaml.FullLoader)
            pos = d['pose']['pose']['position']
            x_points.append(pos['x'])
            y_points.append(pos['y'])
        elif topic == '/metadata':
            d = yaml.load(str(msg), Loader=yaml.FullLoader)
            metadata = json.loads(d['data'])
        elif topic == '/experiment_metrics':
            d = yaml.load(str(msg), Loader=yaml.FullLoader)
            experiment_metrics = json.loads(d['data'])
        elif topic == '/first_image':
            first_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # Determination of the world
    # The world is determined by the metadata
    world = metadata.get('world', 'unknown').split('.')[0]

    # Initialize the dictionary for the world if it doesn't exist
    if world not in all_data:
        all_data[world] = {
            'percentage_completed': [], 'completed_distance': [],
            'lap_seconds': [], 'circuit_diameter': [], 'average_speed': [],
            'image': {'first_images': [], 'path_x': [], 'path_y': []},
            'position_deviation_mae': [], 'position_deviation_total_err': [],
            'mean_brain_iterations_real_time': [], 'brain_iterations_frequency_real_time': [],
            'target_brain_iterations_real_time': [], 'brain_iterations_frequency_simulated_time': [],
            'target_brain_iterations_simulated_time': [], 'mean_inference_time': [],
            'frame_rate': [], 'mean_brain_iterations_simulated_time': [],
            'real_time_factor': [], 'real_time_update_rate': [],
            'experiment_total_simulated_time': [], 'experiment_total_real_time': []
        }

    d = all_data[world]
    # Add data to the dictionary
    d['completed_distance'].append(experiment_metrics.get('completed_distance', 0))
    d['percentage_completed'].append(experiment_metrics.get('percentage_completed', 0))
    d['image']['first_images'].append(first_image)
    d['image']['path_x'].append(x_points)
    d['image']['path_y'].append(y_points)
    d['average_speed'].append(experiment_metrics.get('average_speed', 0))
    d['position_deviation_mae'].append(experiment_metrics.get('position_deviation_mae', 0))
    d['position_deviation_total_err'].append(experiment_metrics.get('position_deviation_total_err', 0))
    d['mean_brain_iterations_real_time'].append(experiment_metrics.get('mean_brain_iterations_real_time', 0))
    d['brain_iterations_frequency_real_time'].append(experiment_metrics.get('brain_iterations_frequency_real_time', 0))
    d['target_brain_iterations_real_time'].append(experiment_metrics.get('target_brain_iterations_real_time', 0))
    d['brain_iterations_frequency_simulated_time'].append(experiment_metrics.get('brain_iterations_frequency_simulated_time', 0))
    d['target_brain_iterations_simulated_time'].append(experiment_metrics.get('target_brain_iterations_simulated_time', 0))
    d['mean_inference_time'].append(experiment_metrics.get('mean_inference_time', 0))
    d['frame_rate'].append(experiment_metrics.get('frame_rate', 0))
    d['mean_brain_iterations_simulated_time'].append(experiment_metrics.get('mean_brain_iterations_simulated_time', 0))
    d['real_time_factor'].append(experiment_metrics.get('real_time_factor', 0))
    d['real_time_update_rate'].append(experiment_metrics.get('real_time_update_rate', 0))
    d['experiment_total_simulated_time'].append(experiment_metrics.get('experiment_total_simulated_time', 0))
    d['experiment_total_real_time'].append(experiment_metrics.get('experiment_total_real_time', 0))
    if 'lap_seconds' in experiment_metrics:
        d['lap_seconds'].append(experiment_metrics['lap_seconds'])
        d['circuit_diameter'].append(experiment_metrics['circuit_diameter'])
    else:
        d['lap_seconds'].append(0.0)
        d['circuit_diameter'].append(0.0)


def process_bag_msgs_ros2(bag_msgs, all_data):
    """
    Processes messages from a ROS2 bag (CARLA) and updates all_data.
    - bag_msgs: list of tuples (topic, msg, timestamp)
    - all_data: global dictionary to update
    """
   
    x_points, y_points = [], []
    collision_count = 0
    lane_invasion_count = 0
    speeds = []
    sim_time = 0.0

    for topic, msg, t in bag_msgs:
        if topic == '/carla/ego_vehicle/odometry':
            pos = msg.pose.pose.position
            x_points.append(pos.x)
            y_points.append(pos.y)
        elif topic == '/carla/ego_vehicle/collision':
            collision_count += 1
        elif topic == '/carla/ego_vehicle/lane_invasion':
            lane_invasion_count += 1
        elif topic == '/carla/ego_vehicle/speedometer':
            speeds.append(msg.data)
        elif topic == '/clock':
            sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    world = 'carla'
    if world not in all_data:
        all_data[world] = {
            'path_x': [], 'path_y': [],
            'collisions': [], 'lane_invasions': [],
            'average_speed': [], 'sim_time': []
        }

    d = all_data[world]
    d['path_x'].append(x_points)
    d['path_y'].append(y_points)
    d['collisions'].append(collision_count)
    d['lane_invasions'].append(lane_invasion_count)
    avg_speed = np.mean(speeds) if speeds else 0.0
    d['average_speed'].append(avg_speed)
    d['sim_time'].append(sim_time)
    
def process_bag_msgs_ros2_extended(bag_msgs, all_data):
    bridge = CvBridge()
    x_points, y_points = [], []
    metadata = {}
    experiment_metrics = {}
    first_image = np.zeros((1,1))
        
    for topic, msg, t in bag_msgs:
        if topic == '/carla/ego_vehicle/odometry':
            pos = msg.pose.pose.position
            x_points.append(pos.x)
            y_points.append(pos.y)
        elif topic == '/metadata':
            metadata = json.loads(msg.data)
        elif topic == '/experiment_metrics':
            experiment_metrics = json.loads(msg.data)
            print("[DEBUG] experiments_metrics leido", experiment_metrics)
        elif topic == '/first_image':
            # first_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            first_image = bridge.imgmsg_to_cv2(msg, encoding='bgr8')

            
    world = metadata.get('world', 'carla').split('.')[0]
    _fill_all_data(world, all_data, x_points, y_points, experiment_metrics, first_image)
    
def _fill_all_data(world, all_data, x_points, y_points, experiment_metrics, first_image):
    if world not in all_data:
        all_data[world] = {
            'percentage_completed': [], 'completed_distance': [],
            'lap_seconds': [], 'circuit_diameter': [], 'average_speed': [],
            'image': {'first_images': [], 'path_x': [], 'path_y': []},
            'position_deviation_mae': [], 'position_deviation_total_err': [],
            'mean_brain_iterations_real_time': [], 'brain_iterations_frequency_real_time': [],
            'target_brain_iterations_real_time': [], 'brain_iterations_frequency_simulated_time': [],
            'target_brain_iterations_simulated_time': [], 'mean_inference_time': [],
            'frame_rate': [], 'mean_brain_iterations_simulated_time': [],
            'real_time_factor': [], 'real_time_update_rate': [],
            'experiment_total_simulated_time': [], 'experiment_total_real_time': []
        }
    d = all_data[world]
    d['completed_distance'].append(experiment_metrics.get('completed_distance', 0))
    d['percentage_completed'].append(experiment_metrics.get('percentage_completed', 0))
    d['image']['first_images'].append(first_image)
    d['image']['path_x'].append(x_points)
    d['image']['path_y'].append(y_points)
    d['average_speed'].append(experiment_metrics.get('average_speed', 0))
    d['position_deviation_mae'].append(experiment_metrics.get('position_deviation_mae', 0))
    d['position_deviation_total_err'].append(experiment_metrics.get('position_deviation_total_err', 0))
    d['mean_brain_iterations_real_time'].append(experiment_metrics.get('mean_brain_iterations_real_time', 0))
    d['brain_iterations_frequency_real_time'].append(experiment_metrics.get('brain_iterations_frequency_real_time', 0))
    d['target_brain_iterations_real_time'].append(experiment_metrics.get('target_brain_iterations_real_time', 0))
    d['brain_iterations_frequency_simulated_time'].append(experiment_metrics.get('brain_iterations_frequency_simulated_time', 0))
    d['target_brain_iterations_simulated_time'].append(experiment_metrics.get('target_brain_iterations_simulated_time', 0))
    d['mean_inference_time'].append(experiment_metrics.get('mean_inference_time', 0))
    d['frame_rate'].append(experiment_metrics.get('frame_rate', 0))
    d['mean_brain_iterations_simulated_time'].append(experiment_metrics.get('mean_brain_iterations_simulated_time', 0))
    d['real_time_factor'].append(experiment_metrics.get('real_time_factor', 0))
    d['real_time_update_rate'].append(experiment_metrics.get('real_time_update_rate', 0))
    d['experiment_total_simulated_time'].append(experiment_metrics.get('experiment_total_simulated_time', 0))
    d['experiment_total_real_time'].append(experiment_metrics.get('experiment_total_real_time', 0))
    if 'lap_seconds' in experiment_metrics:
        d['lap_seconds'].append(experiment_metrics['lap_seconds'])
        d['circuit_diameter'].append(experiment_metrics['circuit_diameter'])
    else:
        d['lap_seconds'].append(0.0)
        d['circuit_diameter'].append(0.0)   
        
def read_ros1(bag_file: str, topics: list):
    """
    Read a ROS1 bag (.bag) and return a list of tuples (topic, msg, timestamp_s).
    """
    import rosbag
    msgs = []
    bag = rosbag.Bag(bag_file)
    for topic, msg, t in bag.read_messages(topics=topics):
        msgs.append((topic, msg, t.to_sec()))
    bag.close()
    return msgs


def read_ros2(bag_dir: str, topics: list, topic_type_map: dict):
    """
    Read a ROS2 bag (.bag/ directory) and returns a list of tuples (topic, msg, timestamp_s).
    - bag_dir: folder containing.db3 and metadata.yaml
    - topics: list of topics to extact
    - topic_type_map: dict {topic: 'package/msg/Type'} for deserialization
    """
    msgs = []
    storage_opts = StorageOptions(uri=bag_dir, storage_id='sqlite3')
    conv_opts    = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader = SequentialReader()
    reader.open(storage_opts, conv_opts)

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in topics:
            continue
        # if we know the type of the topic, deserialize it
        type_str = topic_type_map.get(topic)
        if type_str:
            msg_cls = get_message(type_str)
            msg = deserialize_message(data, msg_cls)
        else:
            msg = data
        msgs.append((topic, msg, t / 1e9))
    return msgs

def save_metrics_to_json(all_data, output_dir):
    """
    Save processed metrics to a JSON file for each world.
    - all_data: dictionary of processed metrics
    - output_dir: base directory to save the JSON files
    """
   
    json_dir = os.path.join(output_dir, 'metrics_json')
    os.makedirs(json_dir, exist_ok=True)

    for world, metrics in all_data.items():
        output_path = os.path.join(json_dir, f"{world}_metrics.json")
        # Convertir datos de tipo numpy a tipo básico (listas normales)
        def clean(obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, dict):
                return {k: clean(v) for k, v in obj.items()}
            elif isinstance(obj, list):
                return [clean(e) for e in obj]
            else:
                return obj

        cleaned_metrics = clean(metrics)
        
        with open(output_path, 'w') as f:
            json.dump(cleaned_metrics, f, indent=4)

    print(f"\n All metrics saved in: {json_dir}")

  
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Analyze Rosbags and Generate Plots', epilog='Enjoy the program! :)')

    parser.add_argument('-i',
                        '--input',
                        type=str,
                        required=True,
                        help='Path to ROS Bag file directory.')

    parser.add_argument('-o',
                        '--output',
                        type=str,
                        required=True,
                        help='Output to plots directory.')

    args = parser.parse_args()

    # bridge = CvBridge()

    baginput = args.input
    output = args.output
    all_data = {}
     
  
    if ros_version == "2":
        bag_dirs = [
            os.path.join(baginput, d) for d in os.listdir(baginput)
            if os.path.isdir(os.path.join(baginput, d)) and os.path.isfile(os.path.join(baginput, d, 'metadata.yaml'))
        ]
        print(f"Detected {len(bag_dirs)} ROS2 bags under {baginput}")

        for bag_dir in bag_dirs:
            print("Reading ROS2 bag:", bag_dir)
            try:
                available = list_topics_ros2(bag_dir)
                topics_to_read = [t for t in topic_type_map if t in available]
                bag_msgs = read_ros2(bag_dir, topics_to_read, topic_type_map)
                process_bag_msgs_ros2_extended(bag_msgs, all_data)
                if '/metadata' in available and '/experiment_metrics' in available:
                    process_bag_msgs_ros2_extended(bag_msgs, all_data)
                else:
                    process_bag_msgs_ros2(bag_msgs, all_data)
            except Exception as e:
                print("Error reading bag:", bag_dir, e)

    else:
        bag_files = [os.path.join(baginput, f) for f in os.listdir(baginput) if f.endswith('.bag')]
        print(f"Detected {len(bag_files)} ROS1 bags under {baginput}")

        for bag_file in bag_files:
            print('Reading bag:', bag_file)
            try:
                bag_msgs = read_ros1(bag_file, ['/F1ROS/odom', '/metadata', '/experiment_metrics', '/first_image'])
                process_bag_msgs_ros1(bag_msgs, all_data)
            except Exception as e:
                print("Error reading bag:", bag_file, e)

    # Save plots
    for world, metrics in all_data.items():
        base_dir = os.path.join(output, 'bag_analysis_plots', world)
        first_dir = os.path.join(base_dir, 'first_images')
        perf_dir = os.path.join(base_dir, 'performances')
        path_dir = os.path.join(base_dir, 'path_followed')
        os.makedirs(first_dir, exist_ok=True)
        os.makedirs(perf_dir, exist_ok=True)
        os.makedirs(path_dir, exist_ok=True)
        
        

        if 'image' in metrics:
            print("Saving first images")
            images = metrics['image']['first_images']
            xs = metrics['image']['path_x']
            ys = metrics['image']['path_y']
            for i, img in enumerate(images, start=1):
                if img is None or img.size == 0:
                    print(f"[WARN] Imagen vacía o nula en ejecución {i}, se omite.")
                    continue
                if np.max(img) == 0:
                    print(f"[WARN] Imagen completamente negra en ejecución {i}, se omite.")
                    continue
                if img.dtype != np.uint8:
                    # Normaliza y convierte a 8-bit para guardar sin problemas
                    img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)  # DEBUG
                    img = img.astype(np.uint8)       # DEBUG
                cv2.imwrite(os.path.join(first_dir, f'Run_{i}.png'), img)
                plt.figure(figsize=(10,5))
                plt.scatter(xs[i-1], ys[i-1], zorder=3)
                plt.title(f'Path in "{world}", run {i}')
                plt.savefig(os.path.join(path_dir, f'Run_{i}.png'))
                plt.close()

        for key, values in metrics.items():
            if key in ('image', 'path_x', 'path_y') or not values:
                continue
            if isinstance(values[0], (list, tuple, np.ndarray)):
                continue
            labels = [f'Run_{i+1}' for i in range(len(values))]
            plt.figure(figsize=(10,5))
            plt.bar(labels, values, width=0.4)
            plt.ylabel(key)
            plt.title(f'Performance in "{world}" — {key}')
            plt.savefig(os.path.join(perf_dir, f'{key}.png'))
            plt.close()
            
    # Save metrics to JSON
    save_metrics_to_json(all_data, output)
    print(f"\n All plots saved in: {output}")
            
        