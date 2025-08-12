import numpy as np

from scipy.optimize import differential_evolution

from orientation.fusion.ekf import EkfFusion

# from orientation import IMU, EkfFusion  # your EKF classes
# from your_module import read_csv        # or however you read data


TARGET_DELTA = 90.0  # You want the final heading to differ from the initial by 90 deg (example)
DT = 0.01  # Timestep used in your EKF updates


def read_csv(filename):
    """Reads CSV data of the form:
       gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,mag_x,mag_y,mag_z
       Returns each column as a numpy array.
    """
    with open(filename, "r") as f:
        lines = f.readlines()[1:]  # skip the header
        gx, gy, gz = [], [], []
        ax, ay, az = [], [], []
        mx, my, mz = [], [], []
        for line in lines:
            data = line.strip().split(",")
            gx.append(float(data[0]))
            gy.append(float(data[1]))
            gz.append(float(data[2]))
            ax.append(float(data[3]))
            ay.append(float(data[4]))
            az.append(float(data[5]))
            mx.append(float(data[6]))
            my.append(float(data[7]))
            mz.append(float(data[8]))
    return (
        np.array(gx),
        np.array(gy),
        np.array(gz),
        np.array(ax),
        np.array(ay),
        np.array(az),
        np.array(mx),
        np.array(my),
        np.array(mz),
    )


def run_filter_once(g_variance, a_variance, m_variance,
                    gx_init, gy_init, gz_init,
                    ax_init, ay_init, az_init,
                    mx_init, my_init, mz_init,
                    gx_main, gy_main, gz_main,
                    ax_main, ay_main, az_main,
                    mx_main, my_main, mz_main,
                    dt=DT, target_delta=TARGET_DELTA):
    """
    Initializes an EKF with the 'initial' data (e.g., samples_initial.csv),
    then runs the filter on the 'main' dataset (samples.csv) using the
    provided variance parameters. Returns the heading error relative
    to 'target_delta'.
    """
    # 1) Initialize the EKF with the "initial" calibration data
    fusion = EkfFusion(
        samples_gyro=np.column_stack((gx_init, gy_init, gz_init)),
        samples_accel=np.column_stack((ax_init, ay_init, az_init)),
        samples_magneto=np.column_stack((mx_init, my_init, mz_init)),
        gyro_variance=g_variance,
        accel_variance=a_variance,
        magneto_variance=m_variance,
        delta_t=dt
    )
    initial_heading = fusion.get_euler()[2]

    # 2) Run updates with the main dataset
    for i in range(len(gx_main)):
        fusion.update(
            np.array([gx_main[i], gy_main[i], gz_main[i]]),
            np.array([ax_main[i], ay_main[i], az_main[i]]),
            np.array([mx_main[i], my_main[i], mz_main[i]]),
            dt
        )

    final_heading = fusion.get_euler()[2]

    # 3) Compute and return error
    error = (initial_heading - final_heading) - target_delta
    return error


def objective(variances,
              gx_init, gy_init, gz_init,
              ax_init, ay_init, az_init,
              mx_init, my_init, mz_init,
              gx_main, gy_main, gz_main,
              ax_main, ay_main, az_main,
              mx_main, my_main, mz_main):
    """
    Objective function for the optimizer:
    - 'variances' is a list/tuple [g_variance, a_variance, m_variance].
    - We return the absolute heading error (lower is better).
    """
    g_var, a_var, m_var = variances

    # In some cases, if the optimizer tries negative or zero,
    # enforce positivity (or clamp) to avoid invalid values.
    if g_var <= 0 or a_var <= 0 or m_var <= 0:
        return 1e6  # large penalty for invalid combos

    error = run_filter_once(
        g_var, a_var, m_var,
        gx_init, gy_init, gz_init,
        ax_init, ay_init, az_init,
        mx_init, my_init, mz_init,
        gx_main, gy_main, gz_main,
        ax_main, ay_main, az_main,
        mx_main, my_main, mz_main
    )

    # We want to minimize the absolute error
    return abs(error)


def auto_tune_variances_differential_evolution():
    """
    Uses SciPy's Differential Evolution to find the best (gyro, accel, mag)
    variances that minimize heading error. This is a global optimization method.
    """
    # 1) Load data only once
    gx_init, gy_init, gz_init, ax_init, ay_init, az_init, mx_init, my_init, mz_init = read_csv("samples_initial.csv")
    gx_main, gy_main, gz_main, ax_main, ay_main, az_main, mx_main, my_main, mz_main = read_csv("sensor_data.csv")

    # 2) Define bounds for each variance: (min, max)
    # You can widen or narrow these as necessary.
    # Typical range might be small for gyro, bigger for accel, etc.,
    # but we keep them uniform for demonstration.
    bounds = [
        (1e-5, 100.0),  # gyro variance
        (1e-5, 1.0),  # accel variance
        (1e-5, 1e7)  # mag variance
    ]

    # 3) Run Differential Evolution
    result = differential_evolution(
        objective,
        bounds=bounds,
        strategy='best1bin',  # a common strategy
        maxiter=50,  # adjust for thoroughness vs. speed
        popsize=15,  # population size
        tol=1e-6,  # stopping tolerance
        mutation=(0.5, 1),  # mutation factor
        recombination=0.7,  # crossover probability
        seed=None,
        polish=True,  # use local search to polish the best solution
        args=(gx_init, gy_init, gz_init,
              ax_init, ay_init, az_init,
              mx_init, my_init, mz_init,
              gx_main, gy_main, gz_main,
              ax_main, ay_main, az_main,
              mx_main, my_main, mz_main),
    )

    # 4) Parse results
    best_g_var, best_a_var, best_m_var = result.x
    best_error = result.fun  # This is the absolute heading error from `objective(...)`

    print("\n======== Optimization Results ========")
    print(f"Status: {result.message}")
    print(f"Best Absolute Error: {best_error:.6f}")
    print(f"Optimal Gyro Variance: {best_g_var}")
    print(f"Optimal Accel Variance: {best_a_var}")
    print(f"Optimal Mag Variance: {best_m_var}")
    print("======================================\n")

    return result


if __name__ == "__main__":
    # Run the differential evolution tuner
    tuning_result = auto_tune_variances_differential_evolution()
