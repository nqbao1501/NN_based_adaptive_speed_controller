import pandas as pd
import numpy as np

# ===============================
# Configuration
# ===============================
DT_FALLBACK = 0.002          # 2 ms if timestamp is missing
SP_THRESHOLD = 5.0           # RPM threshold for detecting active step
MIN_STEP_LENGTH = 200        # samples (~0.4 s minimum)
STEADY_RATIO = 0.3           # last 30% for steady-state metrics
RISE_LOW = 0.1
RISE_HIGH = 0.9
SETTLING_BAND = 0.05


# ===============================
# Dataset Loader
# ===============================
def load_dataset(csv_path):
    df = pd.read_csv(csv_path)

    # Clean column names
    df.columns = [c.strip() for c in df.columns]

    required_cols = {'RPM', 'Setpoint'}
    if not required_cols.issubset(df.columns):
        raise ValueError("CSV must contain at least 'RPM' and 'Setpoint' columns")

    # If timestamp exists, assume it is in milliseconds → convert to seconds
    if 'Timestamp' in df.columns:
        df['Timestamp_s'] = df['Timestamp'].astype(float) * 1e-3
    else:
        df['Timestamp_s'] = np.arange(len(df)) * DT_FALLBACK

    return df


# ===============================
# Step Extraction (0 → SP → 0)
# ===============================
def extract_steps(df):
    steps = []
    current_step = None

    for i in range(len(df)):
        sp = df.loc[i, 'Setpoint']

        if sp > SP_THRESHOLD and current_step is None:
            current_step = {
                'setpoint': sp,
                'start_idx': i
            }

        elif sp <= SP_THRESHOLD and current_step is not None:
            current_step['end_idx'] = i

            if current_step['end_idx'] - current_step['start_idx'] >= MIN_STEP_LENGTH:
                steps.append(current_step)

            current_step = None

    return steps


# ===============================
# Per-Step Metrics
# ===============================
def compute_step_metrics(df, step):
    start = step['start_idx']
    end = step['end_idx']
    sp = step['setpoint']

    segment = df.iloc[start:end]
    rpm = segment['RPM'].values
    t = segment['Timestamp_s'].values
    t = t - t[0]  # relative time

    # ===============================
    # Steady-state error
    # ===============================
    steady_start = int(len(rpm) * (1 - STEADY_RATIO))
    rpm_steady = rpm[steady_start:]
    steady_error = np.mean(np.abs(rpm_steady - sp))

    # ===============================
    # Overshoot
    # ===============================
    peak_rpm = np.max(rpm)
    overshoot = max(0.0, (peak_rpm - sp) / sp * 100.0)

    # ===============================
    # Rise time (10% → 90%)
    # ===============================
    low_th = RISE_LOW * sp
    high_th = RISE_HIGH * sp

    try:
        t_low = t[np.where(rpm >= low_th)[0][0]]
        t_high = t[np.where(rpm >= high_th)[0][0]]
        rise_time = t_high - t_low
    except IndexError:
        rise_time = np.nan

    # ===============================
    # Settling time (±5%, must STAY)
    # ===============================
    band = SETTLING_BAND * sp
    settling_time = np.nan

    for i in range(len(rpm)):
        if np.all(np.abs(rpm[i:] - sp) <= band):
            settling_time = t[i]
            break

    return {
        'Setpoint_RPM': sp,
        'SteadyStateError_RPM': steady_error,
        'Overshoot_percent': overshoot,
        'Peak_RPM': peak_rpm,
        'RiseTime_s': rise_time,
        'SettlingTime_s': settling_time
    }


# ===============================
# Global Metrics
# ===============================
def compute_global_metrics(df):
    error = df['RPM'].values - df['Setpoint'].values

    rmse = np.sqrt(np.mean(error ** 2))
    mae = np.mean(np.abs(error))

    return {
        'Global_RMSE_RPM': rmse,
        'Global_MAE_RPM': mae
    }


# ===============================
# Controller Evaluation
# ===============================
def evaluate_controller(csv_path):
    df = load_dataset(csv_path)
    steps = extract_steps(df)

    step_metrics = [compute_step_metrics(df, step) for step in steps]
    step_df = pd.DataFrame(step_metrics)

    global_metrics = compute_global_metrics(df)

    return step_df, global_metrics


# ===============================
# Run Comparison
# ===============================
pi_steps, pi_global = evaluate_controller("hybrid_no_alpha_6v.csv")
hybrid_steps, hybrid_global = evaluate_controller("hybrid_alpha_6v.csv")

print("\n=== Global Metrics ===")
print("Without alpha Controller:", pi_global)
print("With alpha Controller:", hybrid_global)

print("\n=== Per-Step Metrics (Without alpha) ===")
print(pi_steps)

print("\n=== Per-Step Metrics (With alpha) ===")
print(hybrid_steps)
