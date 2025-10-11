from mcap.reader import make_reader
from abc import ABC, abstractmethod
from tf_transformations import euler_from_quaternion
from pathlib import Path
import csv
import sys
import argparse
import rclpy
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import yaml
from os import path
from ament_index_python.packages import get_package_share_directory
import matplotlib.image as mpimg
from scipy.signal import butter, filtfilt
import matplotlib.ticker as ticker

class TopicHandlerRegistry:
    _registry = {}
    _active_handlers = set()

    @classmethod
    def register(cls, handler_class):
        topic_name = handler_class.TOPIC_NAME
        cls._registry[topic_name] = handler_class

    @classmethod
    def activate_handlers(cls, handler_class):
        cls._active_handlers = {
                cls._registry[handler.TOPIC_NAME] for handler in handler_class
        }

    @classmethod
    def get_handler(cls, topic):
        handler_class = cls._registry.get(topic, None)
        if handler_class and handler_class in cls._active_handlers:
            return handler_class()  
        return None

    @classmethod
    def get_active_handlers(cls):
        return cls._active_handlers

class TopicHandler(ABC):
    TOPIC_NAME = None

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        if cls.TOPIC_NAME is not None:
            TopicHandlerRegistry.register(cls)

    @abstractmethod
    def extract_data(self, msg):
        pass

    @classmethod
    def get_topic_name(cls):
        return cls.TOPIC_NAME
    @classmethod
    def get_under_scored_topic_name(cls):
        return "_".join(cls.TOPIC_NAME.strip("/").split("/"))

    def process_message(self, msg, timestamp):
        data = self.extract_data(msg)
        data["stamp"] = timestamp
        return data

class LocalizationHandler(TopicHandler):
    TOPIC_NAME = "/localization/kinematic_state"

    def extract_data(self, msg):
        e = euler_from_quaternion(
                (
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                )
        )
        return {
                "ekf_x": msg.pose.pose.position.x,
                "ekf_y": msg.pose.pose.position.y,
                "ekf_yaw": e[2],
        }

class LocalizationAccelerationHandler(TopicHandler):
    TOPIC_NAME = "/localization/acceleration"

    def extract_data(self, msg):
        return {
                "loc_acc_x": msg.accel.accel.linear.x,
                "loc_acc_y": msg.accel.accel.linear.y,
        }

class VelocityStatusHandler(TopicHandler):
    TOPIC_NAME = "/vehicle/status/velocity_status"

    def extract_data(self, msg):
        return {"vx": msg.longitudinal_velocity, "vy": msg.lateral_velocity}

class SteeringStatusHandler(TopicHandler):
    TOPIC_NAME = "/vehicle/status/steering_status"

    def extract_data(self, msg):    
        return {"steer": msg.steering_tire_angle}

class ImuDataHandler(TopicHandler):
    TOPIC_NAME = "/sensing/imu/imu_data"

    def extract_data(self, msg):
        return {"gyro_z": msg.angular_velocity.z}

class GnssPoseHandler(TopicHandler):
    TOPIC_NAME = "/sensing/gnss/pose_with_covariance"

    def extract_data(self, msg):
        return {
                "gnss_x": msg.pose.pose.position.x,
                "gnss_y": msg.pose.pose.position.y,
                "gnss_z": msg.pose.pose.position.z,
        }

class AckermannCommandHandler(TopicHandler):
    TOPIC_NAME = "/control/command/control_cmd"

    def extract_data(self, msg):
        return {
                "steering_tire_angle_command": msg.lateral.steering_tire_angle,
                "speed_command": msg.longitudinal.speed,
                "acceleration_command": msg.longitudinal.acceleration,
        }

class ActuationCommandHandler(TopicHandler):
    TOPIC_NAME = "/control/command/actuation_cmd"

    def extract_data(self, msg):
        return {
                "actuation_accel_cmd": msg.actuation.accel_cmd,
                "actuation_brake_cmd": msg.actuation.brake_cmd,
                "actuation_steer_cmd": msg.actuation.steer_cmd,
        }

def read_messages(input_bag, topics: list[str]):
    #with open("rosbag2_2025_09_30_11_43_28_0.mcap", "rb") as f:
    with open(input_bag, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages():
            topic = channel.topic
            data = message.data
            timestamp = message.log_time
            if topic not in topics:
                continue

            msg_type = get_message(schema.name)
            msg = deserialize_message(data, msg_type)
            yield topic, msg, timestamp

        del reader


    
def convert_bag_to_csv(
        input_bag: str,
        active_handlers: list[TopicHandlerRegistry],
        overwrite: bool,
        ):
    current_path = Path.cwd()
    csv_paths = [
            current_path / f"{handler.get_under_scored_topic_name()}.csv"
            for handler in active_handlers
            ]
    if (not overwrite) and all([csv_path.exists() for csv_path in csv_paths]):
        print("csv files already exist")
        return

    print("converting bag to csv")

    active_topics = [handler.get_topic_name() for handler in active_handlers]

    csv_files = {
            topic: open(csv_path, "w") for topic, csv_path in zip(active_topics, csv_paths)
            }
    csv_writers = {}

    def write_to_csv(topic, d: dict):
        if topic not in csv_writers:
            csv_writers[topic] = csv.DictWriter(csv_files[topic], fieldnames=d.keys())
            csv_writers[topic].writeheader()
        csv_writers[topic].writerow(d)

    for topic, msg, timestamp in read_messages(input_bag, active_topics):
        handler = TopicHandlerRegistry.get_handler(topic)
        if handler:
            write_to_csv(topic, handler.process_message(msg, timestamp))

def load_csv(active_handlers: list[TopicHandlerRegistry]):
    dataframes = {}
    current_path = Path.cwd()

    for handler in active_handlers:
        csv_path = current_path / f"{handler.get_under_scored_topic_name()}.csv"
        if not csv_path.exists():
            print(f"{csv_path} not found")
            continue
            #return
        if csv_path.stat().st_size == 0:
            print(f"{csv_path} is empty, skipping")
            continue
        dataframes[handler.get_topic_name()] = pd.read_csv(csv_path)

    return dataframes

def interpolate_dataframes(dataframes: dict):
    longest_key = max(dataframes, key=lambda key: len(dataframes[key]["stamp"]))
    longest_df = dataframes[longest_key]
    reference_stamps = longest_df["stamp"].values

    interpolated_dfs = []

    for _key, df in dataframes.items():
        original_stamps = df["stamp"].values

        interpolated_df = pd.DataFrame({"stamp": reference_stamps})

        for column in df.columns:
            if column != "stamp":
                f = interp1d(
                        original_stamps,
                        df[column].values,
                        bounds_error=False,
                        fill_value=np.nan,
                        )
                interpolated_df[column] = f(reference_stamps) # fill missing value and difference of starting points

        interpolated_dfs.append(interpolated_df)

    combined_df = pd.concat(
            interpolated_dfs, axis=1, join="inner"
            )
    combined_df = combined_df.loc[
            :, ~combined_df.columns.duplicated()
            ]

    combined_df.stamp = (
            combined_df.stamp - combined_df.stamp[0]
            ) / 1e9 # still UNIX timestamp

    return combined_df

def map_to_world(mx, my, origin, size, resolution):
    pgm_mx = int(mx + 0.5)
    pgm_my = size[1] - 1 - int(my + 0.5)
    wx = pgm_mx * resolution + origin[0]
    wy = pgm_my * resolution + origin[1]
    return wx, wy

def plot_map_in_world(ax, map_data):
    map_left_bottom = map_to_world(
            0,
            map_data["size"][1],
            map_data["origin"],
            map_data["size"],
            map_data["resolution"],
            )
    map_right_top = map_to_world(
            map_data["size"][0],
            0,
            map_data["origin"],
            map_data["size"],
            map_data["resolution"],
            )
    extent = [
            map_left_bottom[0],
            map_right_top[0],
            map_left_bottom[1],
            map_right_top[1],
            ]
    ax.imshow(map_data["image_array"], cmap="gray", extent=extent)
    ax.set_xlim([extent[0], extent[1]])
    ax.set_ylim([extent[2], extent[3]])

def load_occupancy_grid_map(map_yaml_path: str):
    base_path = path.dirname(map_yaml_path)
    with open(map_yaml_path, "r") as f:
        map_data = yaml.safe_load(f)

    pgm_file_path = path.join(base_path, map_data["image"])
    image = mpimg.imread(pgm_file_path)
    image_array = np.array(image)
    map_data["size"] = [image_array.shape[1], image_array.shape[0]]
    map_data["image_array"] = image_array

    return map_data

def load_reference_path(reference_path_csv_path: str):
    reference_path_df = pd.read_csv(reference_path_csv_path)
    return reference_path_df

def plot_reference_path(ax, ref_path, df):
    ax.plot(ref_path_df.x_m, ref_path_df.y_m, "--", label="reference path")

def get_index_from_time(df, t_start=None, t_end=None):
    if df.stamp[0] != 0:
        df.stamp = (df.stamp - df.stamp[0]) /1e9

    t0 = 0
    t1 = len(df.stamp)
    if t_start is not None:
        t0 = df[df.stamp >= t_start].index[0]
    if t_end is not None:
        t1 = df[df.stamp <= t_end].index[-1]
    return t0, t1

def compute_gyro_odometry(df):
    gyro_odom_x = np.zeros(len(df))
    gyro_odom_y = np.zeros(len(df))
    gyro_odom_yaw = np.zeros(len(df))

    gyro_odom_x[0] = df.ekf_x[0]
    gyro_odom_y[0] = df.ekf_y[0]
    gyro_odom_yaw[0] = df.ekf_yaw[0]

    dt = np.diff(df.stamp)
    dt = np.insert(dt, 0, 0)

    v = df.vx.values
    omega = df.gyro_z.values

    gyro_odom_yaw[1:] = gyro_odom_yaw[0] + np.cumsum(omega[1:] * dt[1:])

    delta_x = v * np.cos(gyro_odom_yaw) * dt
    delta_y = v * np.sin(gyro_odom_yaw) * dt

    gyro_odom_x[1:] = gyro_odom_x[0] + np.cumsum(delta_x[1:])
    gyro_odom_y[1:] = gyro_odom_y[0] + np.cumsum(delta_y[1:])

    df["gyro_odom_x"] = gyro_odom_x
    df["gyro_odom_y"] = gyro_odom_y
    df["gyro_odom_yaw"] = gyro_odom_yaw

def compensate_gyro_odometry_by_ekf_localization(df, reset_localization_indices: list[int]):
    diff_x = np.diff(df.gyro_odom_x)
    diff_y = np.diff(df.gyro_odom_y)
    diff_yaw = np.diff(df.gyro_odom_yaw)

    reset_points = []

    for i in reset_localization_indices:
        df.gyro_odom_x[i] = df.ekf_x[i]
        df.gyro_odom_x[i + 1 :] = df.gyro_odom_x[i] + np.cumsum(diff_x[i:])
        df.gyro_odom_y[i] = df.ekf_y[i]
        df.gyro_odom_y[i + 1 :] = df.gyro_odom_y[i] + np.cumsum(diff_y[i:])
        df.gyro_odom_yaw[i] = df.ekf_yaw[i]
        df.gyro_odom_yaw[i + 1 :] = df.gyro_odom_yaw[i] + np.cumsum(diff_yaw[i:])

        reset_points.append((df.stamp[i], df.ekf_x[i], df.ekf_y[i], df.ekf_yaw[i]))
    return reset_points

def get_interval(df, duration: float) -> int:
    ave_dt = df.stamp.diff().mean()
    return int(duration / ave_dt)

def m_per_sec_to_kmh(m_per_sec):
    return m_per_sec * 3.6


def plot_trajectory(
        dataframes,
        df,
        t_start=None,
        t_end=None,
        plot_gyro_odom=False,
        plot_gnss=False,
        plot_orientation=False,
        plot_velocity_text=False,
        quiver_skip_duration=1.0,
        velocity_skip_duration=1.0,
        map_yaml_path="",
        reference_path_csv_path="",
        save=False,
        ):
    fig, ax = plt.subplots(1, 1, figsize=(16, 10))

    if map_yaml_path != "":
        plot_map_in_world(ax, load_occupancy_grid_map(map_yaml_path))

    if reference_path_csv_path != "":
        plot_reference_path(ax, load_reference_path(reference_path_csv_path))

    t0, t1 = get_index_from_time(df, t_start, t_end)

    """
    num = (df["ekf_x"] != 0).sum()
    print(num)

    print("ekf_x nonzero count:", (df.ekf_x != 0).sum())
    print("ekf_y nonzero count:", (df.ekf_y != 0).sum())
    print("ekf_x range:", df.ekf_x.min(), "→", df.ekf_x.max())
    print("ekf_y range:", df.ekf_y.min(), "→", df.ekf_y.max())
    print("DataFrame shape:", df.shape)
    """



    ax.plot(df.ekf_x[t0:t1], df.ekf_y[t0:t1], label="ekf")

    if plot_gyro_odom:
        compute_gyro_odometry(df)

        reset_localization_indices = []
        reset_points = compensate_gyro_odometry_by_ekf_localization(df, reset_localization_indices)

        ax.plot(df.gyro_odom_x[t0:t1], df.gyro_odom_y[t0:t1], label="gyro odom")

        if len(reset_points) > 0:
            ax.plot(
                    [p[1] for p in reset_points],
                    [p[2] for p in reset_points],
                    "*",
                    markersize=5.0,
                    label="reset point",
                    )
    if plot_gnss:
        gnss_df = dataframes[GnssPoseHandler.TOPIC_NAME]
        tg0, tg1 = get_index_from_time(gnss_df, t_start, t_end)

        ax.plot(
                gnss_df.gnss_x[tg0:tg1],
                gnss_df.gnss_y[tg0:tg1],
                "o",
                markersize=2.0,
                label="gnss",
                )

    if plot_orientation:
        quiver_skip = get_interval(df, quiver_skip_duration)
        ax.quiver(
                df.ekf_x[t0:t1:quiver_skip],
                df.ekf_y[t0:t1:quiver_skip],
                np.cos(df.ekf_yaw[t0:t1:quiver_skip]),
                np.sin(df.ekf_yaw[t0:t1:quiver_skip]),
                angles="xy",
                scale_units="xy",
                scale=1,
                color="blue",
                label="ekf yaw",
                )
        if plot_gyro_odom:
            ax.quiver(
                    df.gyro_odom_x[t0:t1:quiver_skip],
                    df.gyro_odom_y[t0:t1:quiver_skip],
                    np.cos(df.gyro_odom_yaw[t0:t1:quiver_skip]),
                    np.sin(df.gyro_odom_yaw[t0:t1:quiver_skip]),
                    angles="xy",
                    scale_units="xy",
                    scale=1,
                    color="red",
                    label="gyro odom yaw",
                    )
    if plot_velocity_text:
        velocity_skip = get_interval(df, velocity_skip_duration)
        for i in range(t0, t1, velocity_skip):
            ax.text(
                    df.ekf_x[i],
                    df.ekf_y[i],
                    f"{m_per_sec_to_kmh(df.vx[i]):.1f}",
                    fontsize=6,
                    color="blue",
                    )
    ax.ticklabel_format(useOffset=False, style='plain', axis='both')
    ax.legend()
    ax.grid()
    plt.gca().set_aspect("equal", adjustable="box")

    if save:
        save_plot(fig, f"trajectory", save)
    else:
        plt.show()

    plt.clf()
    plt.close()

def lowpass_filter(data, cutoff_freq, fs, order=5):
    nyquist_freq = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist_freq
    b, a = butter(order, normal_cutoff, btype="low", analog=False)
    y = filtfilt(b, a, data)
    return y

def interactive_compute_slope(ax):
    while True:
        points = plt.ginput(2, timeout=-1)
        if len(points) < 2:
            print("Exiting...")
            break

        x1, y1 = points[0]
        x2, y2 = points[1]
        if x2 != x1:
            slope = (y2 - y1) / (x2 - x1)
            slope_text_str = f"Slope: {slope:.2f}"
        else:
            slope_text_str = "Slope: undefined"

        (line,) = ax.plot([x1, x2], [y1, y2], "r--", linewidth=2)

        slope_text = ax.text(
                (x1 + x2) / 2,
                (y1 + y2) / 2,
                slope_text_str,
                color="blue",
                fontsize=10,
                ha="center",
                )
        plt.draw()



def plot_velocity_acceleration(df, t_start=None, t_end=None, plot_acc=False, save=False):
    t0, t1 = get_index_from_time(df, t_start, t_end)

    dt = np.diff(df.stamp[t0:t1])
    acc_x = np.diff(df.vx[t0:t1]) / dt
    acc_rz = np.diff(df.gyro_z[t0:t1]) / dt

    cutoff_frequency = 1.0
    sampling_rate = 1.0 / np.average(dt)
    acc_x = lowpass_filter(acc_x, cutoff_frequency, sampling_rate)
    acc_rz = lowpass_filter(acc_rz, cutoff_frequency, sampling_rate)

    fig, ax = plt.subplots(2, 1, figsize=(10, 6))

    # accel
    ax[0].plot(df.stamp[t0:t1], 3.6 * df.vx[t0:t1], label="vx [measured]")
    ax[0].plot(df.stamp[t0:t1], 3.6 * df.speed_command[t0:t1], label="speed [cmd]")
    ax[0].plot(df.stamp[t0:t1], 3.6 * df.acceleration_command[t0:t1], label="accel [cmd]")
    ax[0].plot(df.stamp[t0:t1], 3.6 * df.actuation_accel_cmd, label="accel pedal [cmd, 0~1]")
    ax[0].plot(df.stamp[t0:t1], 3.6 * -df.actuation_brake_cmd, label="brake pedal [cmd, 0~1]")


    # steer
    ax[1].plot(df.stamp[t0:t1], df.steer[t0:t1], label="steer [measured]")
    ax[1].plot(df.stamp[t0:t1], df.steering_tire_angle_command[t0:t1], label="steer [cmd]")
    #ax[1].plot(df.stamp[t0:t1], df.actuation_steer_cmd[t0:t1], label="actuator_steer [cmd]") # same value as above
    ax[1].plot(df.stamp[t0:t1], df.gyro_z[t0:t1], label="yaw [measured]")
    ax[1].legend()
    ax[1].yaxis.set_major_locator(ticker.MultipleLocator(1.0))
    ax[1].grid(True, which="both")


    ax[0].set_ylim([-10.0, 40.0])
    #ax[0].set_ylim([-2.0, 10.0])
    ax[0].legend()
    ax[0].xaxis.set_major_locator(ticker.MultipleLocator(10.0))
    ax[0].yaxis.set_major_locator(ticker.MultipleLocator(5.0))
    ax[0].grid(True, which="both")

    if plot_acc:
        ax[1].plot(df.stamp[t0 + 1 : t1], 3.6 * acc_x, label="acc_x [measured]")
        ax[1].plot(df.stamp[t0:t1], 3.6 * df.loc_acc_x[t0:t1], label="loc_acc_x [measured]")
        ax[1].legend()
        ax[1].yaxis.set_major_locator(ticker.MultipleLocator(2.0))
        ax[1].grid(True, which="both")

    interactive_compute_slope(ax[0])
    save_plot(fig, f"velocity_acceleration_{t0}_{t1}", save)
    plt.clf()
    plt.close()

def try_load_mpc_config():
    try:
        mpc_plg_path = get_package_share_directory("multi_purpose_mpc_ros")
        mpc_config_path = Path(mpc_pkg_path) / "config" / "config.yaml"
        with open(mpc_config_path, "r") as f:
            mpc_config = yaml.safe_load(f)
            map_yaml_path = Path(mpc_pkg_path) / Path(mpc_config["map"]["yaml_path"])
            reference_path_csv_path = Path(mpc_pkg_path) / Path(
                    mpc_config["reference_path"]["csv_path"]
                    )
    except:
        """
        aic_tools_path = get_package_share_directory("aic_tools")
        map_yaml_path = (
                Path(aic_tools_path) / "resources" / "map" / "occupancy_grid_map.yaml"
                )
        """
        map_yaml_path = Path.cwd().parent / "occupancy_grid_map.yaml"
        reference_path_csv_path = ""
    return (map_yaml_path, reference_path_csv_path)

def parse_args(argv):
    (OCCUPANCY_GRID_MAP_YAML_PATH, REFERENCE_PATH_CSV_PATH) = try_load_mpc_config()
    parser = argparse.ArgumentParser()
    parser.add_argument(
            "input_bag",
            type=str,
            help="*.mcap file",
            )
    parser.add_argument(
            "-m",
            "--map_yaml_path",
            type=str,
            help="Path to the occupancy grid map yaml file",
            default=OCCUPANCY_GRID_MAP_YAML_PATH,
            )
    parser.add_argument(
            "-w",
            "--overwrite",
            action="store_true",
            help="Overwrite existing csv files",
            )
    args = parser.parse_args(argv)
    return args

def main(argv=sys.argv):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    args = parse_args(args_without_ros[1:])

    TARGET_HANDLERS = [
            LocalizationHandler,
            LocalizationAccelerationHandler,
            VelocityStatusHandler,
            SteeringStatusHandler,
            ImuDataHandler,
            GnssPoseHandler,
            AckermannCommandHandler,
            ActuationCommandHandler,
    ]

    TopicHandlerRegistry.activate_handlers(TARGET_HANDLERS)
    active_handlers = TopicHandlerRegistry.get_active_handlers()

    convert_bag_to_csv(args.input_bag, active_handlers, args.overwrite)

    dataframes = load_csv(active_handlers)

    df = interpolate_dataframes(dataframes)
    print(df.ekf_x)
    print("ekf_x range:", df.ekf_x.max() - df.ekf_x.min())
    print("ekf_y range:", df.ekf_y.max() - df.ekf_y.min())

    """
    plot_trajectory(
            dataframes,
            df,
            plot_gyro_odom=False,
            plot_gnss=True,
            plot_orientation=False,
            plot_velocity_text=True,
            map_yaml_path=args.map_yaml_path,
            #reference_path_csv_path=args.reference_path_csv_path,
            reference_path_csv_path="",
            save=False,
            )
    """

    plot_velocity_acceleration(df, save=False)

    #plot_gnss_and_ekf_with_phase_diff

    rclpy.shutdown()



if __name__ == "__main__":
    main()
