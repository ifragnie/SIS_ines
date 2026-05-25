#!/usr/bin/env python3

# Implemented by: Alice Bocquet, 379741
# Implemented by: Inès Fragnière, 363961


from pathlib import Path
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt


def load_file(filename: Path) -> pd.DataFrame:
    '''
    Load a csv file into a pandas dataframe and strip spaces from column names.
    '''

    df = pd.read_csv(filename, sep=',')

    # Strip spaces from column names
    df.rename(columns=lambda x: x.strip(), inplace=True)

    # Drop empty last column if it exists
    if len(df.columns) > 0 and df.columns[-1].startswith("Unnamed"):
        df = df.drop(df.columns[-1], axis=1)

    # Convert values to numeric when possible
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    # Remove fully empty rows
    df = df.dropna(how="all")

    return df


def get_trajectory_until_stop(df: pd.DataFrame, movement_threshold: float = 1e-4):
    '''
    Cut the ground truth data when the robot stops moving.
    This avoids overestimating trajectory time if the simulation continues after STOP.
    '''

    df = df.copy()

    df["dx"] = df["x"].diff()
    df["dy"] = df["y"].diff()
    df["ds"] = np.sqrt(df["dx"]**2 + df["dy"]**2)

    moving = df["ds"] > movement_threshold

    if moving.any():
        last_moving_index = moving[moving].index[-1]
        df_moving = df.loc[:last_moving_index].copy()
        trajectory_time = df_moving["time"].iloc[-1] - df_moving["time"].iloc[0]
    else:
        df_moving = df.copy()
        trajectory_time = 0.0

    return df_moving, trajectory_time


def count_collisions(filename: Path) -> int:
    '''
    Count collisions from collisions.csv.
    If the file is empty, return zero.
    '''

    if not filename.exists() or filename.stat().st_size == 0:
        return 0

    try:
        df = load_file(filename)
        return len(df.dropna(how="all"))
    except pd.errors.EmptyDataError:
        return 0


def plot_trajectory(df: pd.DataFrame, trajectory_time: float, total_collisions: int, output_file: Path, df_lights: pd.DataFrame = None):
    '''
    Plot the robot trajectory.
    '''

    plt.figure(figsize=(7, 6))

    plt.plot(df["x"], df["y"], linewidth=2, label="Robot trajectory")
    plt.scatter(df["x"].iloc[0], df["y"].iloc[0], s=80, label="Start")
    plt.scatter(df["x"].iloc[-1], df["y"].iloc[-1], s=80, label="Stop")

    if df_lights is not None and not df_lights.empty:
        plot_detected_lights(df_lights)


    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(
        f"Robot trajectory\n"
        f"Trajectory time: {trajectory_time:.1f} s, collisions: {total_collisions}"
    )

    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(output_file, dpi=300)
    plt.show()


def get_sensor_column(df: pd.DataFrame):
    '''
    Find the sensor identifier column.
    Handles both possible names: sensor or sensor_id.
    '''

    if "sensor" in df.columns:
        return "sensor"
    if "sensor_id" in df.columns:
        return "sensor_id"
    return None


def plot_temperature(df: pd.DataFrame, output_file: Path):
    '''
    Plot indoor and outdoor temperature measurements on the same figure.
    '''

    sensor_col = get_sensor_column(df)

    plt.figure(figsize=(9, 5))

    if sensor_col is not None:
        for sensor_id in sorted(df[sensor_col].dropna().unique()):
            sensor_data = df[df[sensor_col] == sensor_id]

            plt.scatter(
                sensor_data["time"],
                sensor_data["T_in"],
                s=14,
                label=f"$T_{{in}}$ sensor {int(sensor_id)}"
            )

            plt.scatter(
                sensor_data["time"],
                sensor_data["T_out"],
                s=14,
                marker="x",
                label=f"$T_{{out}}$ sensor {int(sensor_id)}"
            )
    else:
        plt.scatter(df["time"], df["T_in"], s=14, label="$T_{in}$")
        plt.scatter(df["time"], df["T_out"], s=14, marker="x", label="$T_{out}$")

    plt.xlabel("Time [s]")
    plt.ylabel("Temperature [°C]")
    plt.title("Indoor and outdoor temperature measurements")
    plt.grid(True)
    plt.legend(fontsize=8)
    plt.tight_layout()

    plt.savefig(output_file, dpi=300)
    plt.show()

def plot_detected_lights(df_lights: pd.DataFrame):
    """
    Plot detected lights on the current trajectory figure.

    response_fft:
        1 -> red    -> Low Flashing Defect
        2 -> orange -> Fast Flashing Defect
        3 -> green  -> Good Light
    """

    color_map = {
        1: ("red",    "Low Flashing Defect"),
        2: ("orange", "Fast Flashing Defect"),
        3: ("green",  "Good Light")
    }

    already_labeled = set()

    for _, row in df_lights.iterrows():

        response = int(row["response_fft"])

        if response not in color_map:
            continue

        color, label = color_map[response]

        # Avoid duplicated legend entries
        if label in already_labeled:
            label = None
        else:
            already_labeled.add(label)

        plt.scatter(
            row["x"],
            row["y"],
            color=color,
            s=120,
            edgecolors="black",
            linewidths=1.2,
            marker="o",
            label=label,
            zorder=10
        )


if __name__ == '__main__':

    # This script is located in:
    # material/scripts/python/load_data.py
    #
    # Paths are built relative to the script location, so this works on another
    # computer as long as the material/ folder structure is unchanged.

    SCRIPT_DIR = Path(__file__).resolve().parent
    MATERIAL_DIR = SCRIPT_DIR.parents[1]

    ground_truth_file = MATERIAL_DIR / "controllers" / "supervisor" / "data" / "ground_truth.csv"
    collisions_file = MATERIAL_DIR / "controllers" / "supervisor" / "data" / "collisions.csv"
    temperature_file = MATERIAL_DIR / "controllers" / "controller" / "data" / "temperature.csv"
    lights_file = MATERIAL_DIR/"controllers" /"controller"/"data" /"light.csv"

    output_dir = SCRIPT_DIR / "wp1_plots"
    output_dir_wp3 = SCRIPT_DIR / "wp3_plots"
    output_dir.mkdir(parents=True, exist_ok=True)

    # =====================
    # NAVIGATION
    # =====================

    ground_truth = load_file(ground_truth_file)
    ground_truth_moving, trajectory_time = get_trajectory_until_stop(ground_truth)

    total_collisions = count_collisions(collisions_file)

    plot_trajectory(
        ground_truth_moving,
        trajectory_time,
        total_collisions,
        output_dir / "trajectory_ground_truth.png"
    )

    # =====================
    # TEMPERATURE
    # =====================

    if temperature_file.exists() and temperature_file.stat().st_size > 0:
        temperature = load_file(temperature_file)

        if "time" in temperature.columns and "T_in" in temperature.columns and "T_out" in temperature.columns:
            plot_temperature(
                temperature,
                output_dir / "temperature_in_out.png"
            )
        else:
            print("Temperature file found, but required columns are missing.")
            print("Columns found:", temperature.columns.tolist())
    else:
        print("No temperature file found or file is empty:", temperature_file)

    # =====================
    # LIGHTS
    # =====================

    if lights_file.exists() and lights_file.stat().st_size > 0:
        light = load_file(lights_file)

        plot_trajectory(ground_truth_moving, trajectory_time, total_collisions, output_dir_wp3 / "trajectory_and_lights.png", light)
    else:
        print("Not light file found or file is empty.")

    # =====================
    # SUMMARY
    # =====================

    summary_file = output_dir / "wp1_summary.txt"

    with open(summary_file, "w") as f:
        f.write("WP1 summary\n")
        f.write("===========\n")
        f.write(f"Trajectory time until stop: {trajectory_time:.2f} s\n")
        f.write(f"Total collisions: {total_collisions}\n")
        f.write(f"Ground truth file: {ground_truth_file}\n")
        f.write(f"Collisions file: {collisions_file}\n")
        f.write(f"Temperature file: {temperature_file}\n")

    print("\nDone.")
    print(f"Trajectory time until stop: {trajectory_time:.2f} s")
    print(f"Total collisions: {total_collisions}")
    print("Plots saved in:", output_dir)
    print("Summary saved in:", summary_file)
    

def wrap_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def plot_trajectory(gt, ekf):
    plt.figure(figsize=(8, 6))

    plt.plot(gt["x"], gt["y"],
             linewidth=2, label="Ground Truth", color="blue")

    plt.plot(ekf["est_x"], ekf["est_y"],
             "--", linewidth=2, label="EKF Estimation", color="orange")

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Robot Trajectory — Ground Truth vs EKF Estimation")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")


def plot_position_errors(time, error_x, error_y, sigma_x, sigma_y, scale=1.0):
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))

    # X
    axs[0].plot(time, error_x, linewidth=1.5, label="Error x", color="blue")
    axs[0].plot(time,  scale * sigma_x, "--", color="orange", label=f"+{scale}σ_x")
    axs[0].plot(time, -scale * sigma_x, "--", color="orange", label=f"-{scale}σ_x")
    axs[0].set_xlabel("time [s]")
    axs[0].set_ylabel("x error [m]")
    axs[0].set_title(f"Position Error in X (uncertainty scaled ×{scale})")
    axs[0].legend()
    axs[0].grid(True)

    # Y
    axs[1].plot(time, error_y, linewidth=1.5, label="Error y", color="blue")
    axs[1].plot(time,  scale * sigma_y, "--", color="orange", label=f"+{scale}σ_y")
    axs[1].plot(time, -scale * sigma_y, "--", color="orange", label=f"-{scale}σ_y")
    axs[1].set_xlabel("time [s]")
    axs[1].set_ylabel("y error [m]")
    axs[1].set_title(f"Position Error in Y (uncertainty scaled ×{scale})")
    axs[1].legend()
    axs[1].grid(True)

    fig.suptitle("Position Errors and Uncertainty")
    plt.tight_layout()


def plot_heading_error(time, error_theta, sigma_theta, scale=1.0):
    plt.figure(figsize=(10, 4))

    plt.plot(time, error_theta, linewidth=1.5, label="Heading Error", color="blue")
    plt.plot(time,  scale * sigma_theta, "--", color="orange", label=f"+{scale}σ_θ")
    plt.plot(time, -scale * sigma_theta, "--", color="orange", label=f"-{scale}σ_θ")

    plt.xlabel("time [s]")
    plt.ylabel("heading error [rad]")
    plt.title(f"Heading Error and Uncertainty (uncertainty scaled ×{scale})")
    plt.legend()
    plt.grid(True)


if __name__ == "__main__":

    # ---- Chemins des fichiers ----
    gt = load_file("controllers/supervisor/data/ground_truth.csv")
    ekf = load_file("controllers/controller/data/ekf_estimation.csv")

    print("Ground truth columns:", gt.columns.tolist())
    print("EKF columns:         ", ekf.columns.tolist())
    print(gt.head())
    print(ekf.head())

    # ---- Interpolation du ground truth sur les timestamps EKF ----
    gt_x     = np.interp(ekf["time"], gt["time"], gt["x"])
    gt_y     = np.interp(ekf["time"], gt["time"], gt["y"])
    gt_theta = np.interp(ekf["time"], gt["time"], gt["heading"])

    # ---- Erreurs ----
    error_x     = ekf["est_x"]     - gt_x
    error_y     = ekf["est_y"]     - gt_y
    error_theta = wrap_angle(ekf["est_theta"] - gt_theta)

    # ---- Incertitudes (sqrt des variances loggées) ----
    sigma_x     = np.sqrt(np.abs(ekf["sigma_x"]))
    sigma_y     = np.sqrt(np.abs(ekf["sigma_y"]))
    sigma_theta = np.sqrt(np.abs(ekf["sigma_theta"]))

    # ---- Echelle pour la visualisation ----
    # Ajuster si les sigma sont trop petits par rapport aux erreurs
    SCALE = 1.0  # changer à 5, 10, etc. si nécessaire

    # ---- Plots ----
    plot_trajectory(gt, ekf)
    plot_position_errors(ekf["time"], error_x, error_y,
                         sigma_x, sigma_y, scale=SCALE)
    plot_heading_error(ekf["time"], error_theta, sigma_theta, scale=SCALE)

    plt.show()