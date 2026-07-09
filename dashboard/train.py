# train.py
# PAI-Car Ridge Linear Regression model training + generated model self-test
#
# 기능:
#   1. logs 폴더의 pai_car_pyqt_*.csv telemetry 파일만 읽는다.
#   2. section_marks_*.csv 오선택을 막는다.
#   3. 학습에 적합한 row만 필터링한다.
#   4. Ridge Linear Regression으로 left_cmd, right_cmd를 학습한다.
#   5. Pico에서 import 가능한 paicar_lr_model.py를 저장한다.
#   6. 저장된 모델을 다시 import해서 샘플 예측 테스트까지 수행한다.
#
# 실행:
#   python train.py
#
# 주의:
#   - 이 모델은 현재 주행 코드의 left_cmd/right_cmd를 모방하는 모델이다.
#   - lap time 최적화 모델이 아니다.
#   - 실차 적용 전에는 반드시 기존 PD 제어기 fallback을 유지하라.

import glob
import importlib.util
import math
import os
import sys

import numpy as np
import pandas as pd


# ------------------------------------------------------------
# Settings
# ------------------------------------------------------------

# dashboard/train.py 기준이면 logs/를 사용한다.
# 테스트 편의를 위해 logs/에 파일이 없으면 현재 폴더도 검색한다.
LOG_DIRS = ["logs", "."]

# 특정 파일 하나만 직접 지정하고 싶으면 사용한다.
# 예:
# CSV_FILE = "logs/pai_car_pyqt_20260710_062126.csv"
CSV_FILE = None

# 여러 파일을 직접 지정하고 싶으면 사용한다.
# CSV_FILES가 비어 있으면 LOG_DIRS에서 자동 검색한다.
CSV_FILES = []

# True: profile snapshot이 들어있는 모든 pai_car_pyqt_*.csv 사용
# False: 최신 파일 1개만 사용
USE_ALL_CSV = True

# profile snapshot 컬럼이 있는 CSV만 사용한다.
# app.py 수정 이후 생성된 CSV만 학습 대상으로 삼기 위해 True 권장.
REQUIRE_PROFILE_SNAPSHOT = True

MODEL_OUT = "paicar_lr_model.py"

RIDGE_ALPHA = 10.0
TEST_RATIO = 0.2

# 학습 필터
EXCLUDE_MARKER = True
EXCLUDE_OFF_LINE = True
REQUIRE_RUN_STATE_RUN = True
REQUIRE_SECTION_SYNC = True
REQUIRE_TELEMETRY_V2 = True
REQUIRE_POSITIVE_BASE_SPEED = True

# Pico 모델 출력 제한
# 첫 실차 테스트가 불안하면 MOTOR_MIN = 0으로 바꿔서 역회전을 막아라.
MOTOR_MIN = -300
MOTOR_MAX = 1000

# 자체 테스트 설정
SELF_TEST_SAMPLE_COUNT = 20
SELF_TEST_MAX_INTERNAL_DIFF = 1e-6


# ------------------------------------------------------------
# Feature / target columns
# ------------------------------------------------------------

BASE_FEATURE_COLUMNS = [
    "n0", "n1", "n2", "n3", "n4", "n5", "n6", "n7",
    "position",
    "error",
    "d_error",
    "base_speed",
]

OPTIONAL_FEATURE_COLUMNS = [
    "actual_section_id",
    "active_profile_id",
    "profile_version",
    "profile_base_speed",
    "profile_curve_speed",
    "profile_sharp_curve_speed",
    "profile_min_run_speed",
    "profile_kp",
    "profile_kd",
    "profile_max_correction",
    "profile_reverse_allow",
    "profile_reverse_pwm_mid",
    "profile_reverse_pwm_high",
    "profile_error_curve_threshold",
    "profile_error_sharp_threshold",
    "profile_d_error_curve_threshold",
    "profile_d_error_sharp_threshold",
    "profile_search_pwm",
    "profile_line_loss_max_ms",
]

PROFILE_SNAPSHOT_COLUMNS = [
    "profile_set",
    "profile_version",
    "profile_source_key",
    "profile_base_speed",
    "profile_curve_speed",
    "profile_sharp_curve_speed",
    "profile_kp",
    "profile_kd",
]

TARGET_COLUMNS = [
    "left_cmd",
    "right_cmd",
]


# ------------------------------------------------------------
# CSV discovery / loading
# ------------------------------------------------------------

def normalize_path(path):
    return os.path.normpath(os.path.abspath(path))


def find_telemetry_csv_files(log_dirs):
    files = []

    for log_dir in log_dirs:
        pattern = os.path.join(log_dir, "pai_car_pyqt_*.csv")
        files.extend(glob.glob(pattern))

    # 중복 제거
    unique = []
    seen = set()

    for path in files:
        full = normalize_path(path)
        if full not in seen:
            seen.add(full)
            unique.append(path)

    if not unique:
        patterns = [os.path.join(d, "pai_car_pyqt_*.csv") for d in log_dirs]
        raise FileNotFoundError(
            "No telemetry CSV files found. Expected patterns: {}".format(patterns)
        )

    unique.sort(key=os.path.getmtime)
    return unique


def file_has_required_columns(path, required_columns):
    try:
        header = pd.read_csv(path, nrows=0).columns
    except Exception:
        return False

    return all(col in header for col in required_columns)


def select_csv_files():
    if CSV_FILE is not None:
        return [CSV_FILE]

    if CSV_FILES:
        return list(CSV_FILES)

    files = find_telemetry_csv_files(LOG_DIRS)

    if REQUIRE_PROFILE_SNAPSHOT:
        files = [
            path for path in files
            if file_has_required_columns(path, PROFILE_SNAPSHOT_COLUMNS)
        ]

        if not files:
            raise FileNotFoundError(
                "No pai_car_pyqt_*.csv with profile snapshot columns found. "
                "Run the updated dashboard/app.py first."
            )

    if USE_ALL_CSV:
        return files

    return [files[-1]]


def load_csv_files(paths):
    frames = []

    print("CSV files:")

    for idx, path in enumerate(paths):
        print("  [{}] {}".format(idx, path))
        df = pd.read_csv(path)
        df["run_id"] = os.path.basename(path)
        df["run_index"] = idx
        df["source_csv"] = path
        frames.append(df)

    if not frames:
        raise ValueError("No CSV data loaded.")

    return pd.concat(frames, ignore_index=True)


# ------------------------------------------------------------
# Data conversion / filtering
# ------------------------------------------------------------

def bool_to_number(value):
    if value is True:
        return 1
    if value is False:
        return 0

    if isinstance(value, str):
        v = value.strip().lower()
        if v in ("true", "t", "yes", "y"):
            return 1
        if v in ("false", "f", "no", "n"):
            return 0

    return value


def coerce_numeric_column(series):
    return pd.to_numeric(series.map(bool_to_number), errors="coerce")


def build_feature_columns(df):
    missing_base = [col for col in BASE_FEATURE_COLUMNS if col not in df.columns]
    if missing_base:
        raise ValueError("Missing base feature columns: {}".format(missing_base))

    missing_target = [col for col in TARGET_COLUMNS if col not in df.columns]
    if missing_target:
        raise ValueError("Missing target columns: {}".format(missing_target))

    if REQUIRE_PROFILE_SNAPSHOT:
        missing_snapshot = [col for col in PROFILE_SNAPSHOT_COLUMNS if col not in df.columns]
        if missing_snapshot:
            raise ValueError("Missing profile snapshot columns: {}".format(missing_snapshot))

    feature_columns = list(BASE_FEATURE_COLUMNS)

    for col in OPTIONAL_FEATURE_COLUMNS:
        if col in df.columns:
            feature_columns.append(col)

    return feature_columns


def filter_data(df, feature_columns):
    before = len(df)

    numeric_columns = list(feature_columns) + list(TARGET_COLUMNS)

    filter_columns = [
        "telemetry_version",
        "run_state",
        "section_mismatch",
        "on_line",
        "is_marker",
        "base_speed",
        "actual_section_id",
    ]

    for col in filter_columns:
        if col in df.columns and col not in numeric_columns:
            numeric_columns.append(col)

    for col in numeric_columns:
        if col in df.columns:
            df[col] = coerce_numeric_column(df[col])

    # feature/target이 비어 있는 row 제거
    df = df.dropna(subset=feature_columns + TARGET_COLUMNS)

    if REQUIRE_TELEMETRY_V2 and "telemetry_version" in df.columns:
        df = df[df["telemetry_version"] == 2]

    if REQUIRE_RUN_STATE_RUN and "run_state" in df.columns:
        df = df[df["run_state"] == 1]

    if REQUIRE_SECTION_SYNC and "section_mismatch" in df.columns:
        df = df[df["section_mismatch"] == 0]

    if EXCLUDE_OFF_LINE and "on_line" in df.columns:
        df = df[df["on_line"] == 1]

    if EXCLUDE_MARKER and "is_marker" in df.columns:
        df = df[df["is_marker"] == 0]

    if REQUIRE_POSITIVE_BASE_SPEED and "base_speed" in df.columns:
        df = df[df["base_speed"] > 0]

    if "actual_section_id" in df.columns:
        df = df[df["actual_section_id"].between(0, 11)]

    df = df.reset_index(drop=True)

    after = len(df)
    print()
    print("Rows before filter:", before)
    print("Rows after  filter:", after)

    if after < 50:
        raise ValueError("Too few rows for training. Collect more driving data.")

    return df


# ------------------------------------------------------------
# Split
# ------------------------------------------------------------

def split_train_test_by_run_or_time(df, test_ratio=0.2):
    run_ids = list(df["run_id"].drop_duplicates())

    if len(run_ids) >= 2:
        n_test = max(1, int(math.ceil(len(run_ids) * test_ratio)))
        test_runs = set(run_ids[-n_test:])

        train_mask = ~df["run_id"].isin(test_runs)
        test_mask = df["run_id"].isin(test_runs)

        print()
        print("Split mode: run-level split")
        print("Train runs:", [r for r in run_ids if r not in test_runs])
        print("Test runs :", [r for r in run_ids if r in test_runs])

    else:
        n = len(df)
        split_idx = int(n * (1.0 - test_ratio))

        train_mask = np.zeros(n, dtype=bool)
        test_mask = np.zeros(n, dtype=bool)
        train_mask[:split_idx] = True
        test_mask[split_idx:] = True

        print()
        print("Split mode: time-block split")
        print("Train rows: first {} rows".format(split_idx))
        print("Test rows : last {} rows".format(n - split_idx))

    train_df = df[train_mask].reset_index(drop=True)
    test_df = df[test_mask].reset_index(drop=True)

    if len(train_df) < 20 or len(test_df) < 10:
        raise ValueError(
            "Train/test split produced too few rows. train={}, test={}".format(
                len(train_df), len(test_df)
            )
        )

    return train_df, test_df


# ------------------------------------------------------------
# Model
# ------------------------------------------------------------

def fit_standardizer(X):
    mean = np.mean(X, axis=0)
    scale = np.std(X, axis=0)
    scale[scale < 1e-9] = 1.0
    return mean, scale


def apply_standardizer(X, mean, scale):
    return (X - mean) / scale


def train_ridge_regression_numpy(X, y, alpha=10.0):
    ones = np.ones((X.shape[0], 1), dtype=np.float64)
    Xb = np.hstack([ones, X])

    n_features = Xb.shape[1]
    I = np.eye(n_features, dtype=np.float64)
    I[0, 0] = 0.0  # intercept는 regularization 제외

    A = Xb.T @ Xb + alpha * I
    B = Xb.T @ y

    try:
        W = np.linalg.solve(A, B)
    except np.linalg.LinAlgError:
        W, _, _, _ = np.linalg.lstsq(A, B, rcond=None)

    intercept = W[0, :]
    coef = W[1:, :].T
    return coef, intercept


def predict_linear(X, coef, intercept):
    return X @ coef.T + intercept


def calc_metrics(y_true, y_pred):
    err = y_pred - y_true
    mae = np.mean(np.abs(err), axis=0)
    rmse = np.sqrt(np.mean(err ** 2, axis=0))

    ss_res = np.sum(err ** 2, axis=0)
    ss_tot = np.sum((y_true - np.mean(y_true, axis=0)) ** 2, axis=0)

    r2 = np.empty_like(ss_res, dtype=np.float64)
    for i in range(len(ss_res)):
        if ss_tot[i] <= 1e-12:
            r2[i] = np.nan
        else:
            r2[i] = 1.0 - ss_res[i] / ss_tot[i]

    return mae, rmse, r2


def calc_saturation_stats(df):
    left = df["left_cmd"].to_numpy(dtype=np.float64)
    right = df["right_cmd"].to_numpy(dtype=np.float64)

    return {
        "left_min": float(np.min(left)),
        "left_max": float(np.max(left)),
        "right_min": float(np.min(right)),
        "right_max": float(np.max(right)),
        "left_reverse_rate": float(np.mean(left < 0.0)),
        "right_reverse_rate": float(np.mean(right < 0.0)),
        "left_high_sat_rate": float(np.mean(left >= 1000.0)),
        "right_high_sat_rate": float(np.mean(right >= 1000.0)),
    }


def print_metrics(name, y_true, y_pred):
    mae, rmse, r2 = calc_metrics(y_true, y_pred)
    print()
    print("{} MAE left/right : {}".format(name, mae))
    print("{} RMSE left/right: {}".format(name, rmse))
    print("{} R2 left/right  : {}".format(name, r2))
    return mae, rmse, r2


def print_section_metrics(df_part, y_true, y_pred, title):
    if "actual_section_id" not in df_part.columns:
        return

    temp = df_part.copy()
    err = y_pred - y_true
    temp["abs_err_left"] = np.abs(err[:, 0])
    temp["abs_err_right"] = np.abs(err[:, 1])

    grouped = temp.groupby("actual_section_id").agg(
        rows=("actual_section_id", "count"),
        mae_left=("abs_err_left", "mean"),
        mae_right=("abs_err_right", "mean"),
        mean_abs_line_error=("error", lambda s: float(np.mean(np.abs(s)))),
        reverse_left_rate=("left_cmd", lambda s: float(np.mean(s < 0.0))),
        reverse_right_rate=("right_cmd", lambda s: float(np.mean(s < 0.0))),
    )

    print()
    print(title)
    print(grouped.to_string())


# ------------------------------------------------------------
# Save Pico-compatible model
# ------------------------------------------------------------

def fmt_float_values(values):
    return ", ".join("{:.12g}".format(float(v)) for v in values)


def save_model_py(
    model_path,
    feature_columns,
    target_columns,
    coef,
    intercept,
    feature_mean,
    feature_scale,
    alpha,
    motor_min,
    motor_max,
):
    with open(model_path, "w", encoding="utf-8") as f:
        f.write("# paicar_lr_model.py\n")
        f.write("# Auto-generated by train.py\n")
        f.write("# Ridge Linear Regression model for PAI-Car\n")
        f.write("# Predicts left_cmd, right_cmd from telemetry/profile features.\n\n")

        f.write("MODEL_KIND = 'ridge_linear_regression'\n")
        f.write("RIDGE_ALPHA = {:.12g}\n".format(float(alpha)))
        f.write("MOTOR_MIN = {}\n".format(int(motor_min)))
        f.write("MOTOR_MAX = {}\n\n".format(int(motor_max)))

        f.write("FEATURE_COLUMNS = {}\n".format(repr(feature_columns)))
        f.write("TARGET_COLUMNS = {}\n\n".format(repr(target_columns)))

        f.write("FEATURE_MEAN = [\n    ")
        f.write(fmt_float_values(feature_mean))
        f.write("\n]\n\n")

        f.write("FEATURE_SCALE = [\n    ")
        f.write(fmt_float_values(feature_scale))
        f.write("\n]\n\n")

        f.write("COEF = [\n")
        for row in coef:
            f.write("    [")
            f.write(fmt_float_values(row))
            f.write("],\n")
        f.write("]\n\n")

        f.write("INTERCEPT = [")
        f.write(fmt_float_values(intercept))
        f.write("]\n\n")

        f.write("def _to_float(v, default=0.0):\n")
        f.write("    try:\n")
        f.write("        if v is True:\n")
        f.write("            return 1.0\n")
        f.write("        if v is False:\n")
        f.write("            return 0.0\n")
        f.write("        return float(v)\n")
        f.write("    except Exception:\n")
        f.write("        return default\n\n")

        f.write("def _clamp(v, vmin, vmax):\n")
        f.write("    if v > vmax:\n")
        f.write("        return vmax\n")
        f.write("    if v < vmin:\n")
        f.write("        return vmin\n")
        f.write("    return v\n\n")

        f.write("def _get_feature(features, index, name):\n")
        f.write("    if isinstance(features, dict):\n")
        f.write("        return _to_float(features.get(name, 0.0))\n")
        f.write("    if index >= len(features):\n")
        f.write("        return 0.0\n")
        f.write("    return _to_float(features[index])\n\n")

        f.write("def predict_raw(features):\n")
        f.write("    \"\"\"\n")
        f.write("    features can be either:\n")
        f.write("      1. list/tuple in FEATURE_COLUMNS order\n")
        f.write("      2. dict with keys in FEATURE_COLUMNS\n")
        f.write("    returns float left_cmd, right_cmd without clamp\n")
        f.write("    \"\"\"\n")
        f.write("    out = []\n")
        f.write("    for j in range(len(COEF)):\n")
        f.write("        y = INTERCEPT[j]\n")
        f.write("        for i in range(len(FEATURE_COLUMNS)):\n")
        f.write("            name = FEATURE_COLUMNS[i]\n")
        f.write("            x = _get_feature(features, i, name)\n")
        f.write("            x = (x - FEATURE_MEAN[i]) / FEATURE_SCALE[i]\n")
        f.write("            y += COEF[j][i] * x\n")
        f.write("        out.append(y)\n")
        f.write("    return out[0], out[1]\n\n")

        f.write("def predict(features, motor_min=MOTOR_MIN, motor_max=MOTOR_MAX):\n")
        f.write("    \"\"\"returns int left_cmd, right_cmd with clamp\"\"\"\n")
        f.write("    left, right = predict_raw(features)\n")
        f.write("    left = _clamp(left, motor_min, motor_max)\n")
        f.write("    right = _clamp(right, motor_min, motor_max)\n")
        f.write("    return int(left), int(right)\n")

    print()
    print("Model saved:", model_path)


# ------------------------------------------------------------
# Generated model self-test
# ------------------------------------------------------------

def import_module_from_path(module_name, path):
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError("Cannot load module spec from {}".format(path))

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def clamp_array(y, motor_min, motor_max):
    return np.clip(y, motor_min, motor_max)


def run_generated_model_self_test(
    model_path,
    test_df,
    feature_columns,
    coef,
    intercept,
    feature_mean,
    feature_scale,
):
    print()
    print("Generated model self-test:")

    if not os.path.exists(model_path):
        raise AssertionError("Model file was not created: {}".format(model_path))

    model = import_module_from_path("paicar_lr_model_selftest", model_path)

    if list(model.FEATURE_COLUMNS) != list(feature_columns):
        raise AssertionError("FEATURE_COLUMNS mismatch between train.py and generated model")

    n = min(SELF_TEST_SAMPLE_COUNT, len(test_df))
    sample_df = test_df.iloc[:n]
    X = sample_df[feature_columns].to_numpy(dtype=np.float64)
    X_scaled = apply_standardizer(X, feature_mean, feature_scale)
    y_internal_raw = predict_linear(X_scaled, coef, intercept)

    max_raw_diff = 0.0

    for row_index in range(n):
        feature_list = [float(X[row_index, i]) for i in range(len(feature_columns))]
        feature_dict = {
            feature_columns[i]: feature_list[i]
            for i in range(len(feature_columns))
        }

        # list 입력 테스트
        raw_list = model.predict_raw(feature_list)
        # dict 입력 테스트
        raw_dict = model.predict_raw(feature_dict)
        # clamp/int 테스트
        pred_int = model.predict(feature_dict)

        for value in raw_list + raw_dict:
            if not math.isfinite(float(value)):
                raise AssertionError("Non-finite raw prediction: {}".format(value))

        if not isinstance(pred_int[0], int) or not isinstance(pred_int[1], int):
            raise AssertionError("predict() must return int tuple, got {}".format(pred_int))

        if pred_int[0] < MOTOR_MIN or pred_int[0] > MOTOR_MAX:
            raise AssertionError("left_cmd out of clamp range: {}".format(pred_int[0]))

        if pred_int[1] < MOTOR_MIN or pred_int[1] > MOTOR_MAX:
            raise AssertionError("right_cmd out of clamp range: {}".format(pred_int[1]))

        diff_list = max(
            abs(float(raw_list[0]) - float(y_internal_raw[row_index, 0])),
            abs(float(raw_list[1]) - float(y_internal_raw[row_index, 1])),
        )
        diff_dict = max(
            abs(float(raw_dict[0]) - float(y_internal_raw[row_index, 0])),
            abs(float(raw_dict[1]) - float(y_internal_raw[row_index, 1])),
        )
        max_raw_diff = max(max_raw_diff, diff_list, diff_dict)

    if max_raw_diff > SELF_TEST_MAX_INTERNAL_DIFF:
        raise AssertionError(
            "Generated model raw prediction differs from internal prediction. "
            "max_diff={}".format(max_raw_diff)
        )

    print("  import check       : OK")
    print("  feature check      : OK")
    print("  list predict_raw   : OK")
    print("  dict predict_raw   : OK")
    print("  clamp predict      : OK")
    print("  max raw diff       : {:.3e}".format(max_raw_diff))
    print("  samples tested     : {}".format(n))


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

def main():
    csv_paths = select_csv_files()
    df = load_csv_files(csv_paths)

    feature_columns = build_feature_columns(df)
    df = filter_data(df, feature_columns)

    train_df, test_df = split_train_test_by_run_or_time(df, TEST_RATIO)

    X_train = train_df[feature_columns].to_numpy(dtype=np.float64)
    y_train = train_df[TARGET_COLUMNS].to_numpy(dtype=np.float64)

    X_test = test_df[feature_columns].to_numpy(dtype=np.float64)
    y_test = test_df[TARGET_COLUMNS].to_numpy(dtype=np.float64)

    feature_mean, feature_scale = fit_standardizer(X_train)
    X_train_scaled = apply_standardizer(X_train, feature_mean, feature_scale)
    X_test_scaled = apply_standardizer(X_test, feature_mean, feature_scale)

    coef, intercept = train_ridge_regression_numpy(
        X_train_scaled,
        y_train,
        alpha=RIDGE_ALPHA,
    )

    y_train_pred = predict_linear(X_train_scaled, coef, intercept)
    y_test_pred = predict_linear(X_test_scaled, coef, intercept)

    print()
    print("Model: Ridge Linear Regression")
    print("Ridge alpha:", RIDGE_ALPHA)
    print("Feature count:", len(feature_columns))
    print("Target count :", len(TARGET_COLUMNS))

    print()
    print("Features:")
    for i, col in enumerate(feature_columns):
        print("  {:02d}: {}".format(i, col))

    print()
    print("Targets:", TARGET_COLUMNS)
    print("Train rows:", len(train_df))
    print("Test rows :", len(test_df))

    _, _, _ = print_metrics("Train", y_train, y_train_pred)
    test_mae, test_rmse, test_r2 = print_metrics("Test ", y_test, y_test_pred)

    print()
    print("Train target saturation stats:")
    for key, value in calc_saturation_stats(train_df).items():
        print("  {}: {:.4f}".format(key, value))

    print()
    print("Test target saturation stats:")
    for key, value in calc_saturation_stats(test_df).items():
        print("  {}: {:.4f}".format(key, value))

    print_section_metrics(
        test_df,
        y_test,
        y_test_pred,
        "Test section metrics:",
    )

    print()
    print("Intercept:")
    print(intercept)

    print()
    print("Coef shape:", coef.shape)

    save_model_py(
        MODEL_OUT,
        feature_columns,
        TARGET_COLUMNS,
        coef,
        intercept,
        feature_mean,
        feature_scale,
        RIDGE_ALPHA,
        MOTOR_MIN,
        MOTOR_MAX,
    )

    run_generated_model_self_test(
        MODEL_OUT,
        test_df,
        feature_columns,
        coef,
        intercept,
        feature_mean,
        feature_scale,
    )

    print()
    print("Done.")
    print("Model output:", MODEL_OUT)
    print()
    print("Evaluation guide:")
    print("  Test MAE left/right < 80   : good imitation")
    print("  Test MAE 80~150            : usable for bench test, risky on car")
    print("  Test MAE > 150             : not recommended for real driving")
    print("  High section 4/5/8 MAE     : do not use ML on curve sections yet")

    return {
        "test_mae": test_mae,
        "test_rmse": test_rmse,
        "test_r2": test_r2,
        "feature_columns": feature_columns,
        "train_rows": len(train_df),
        "test_rows": len(test_df),
    }


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print()
        print("ERROR:", exc)
        sys.exit(1)