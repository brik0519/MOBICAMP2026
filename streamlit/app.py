# app.py
# PAI-Car CSV Log Viewer
#
# 설치:
#   pip install streamlit pandas plotly
#
# 실행:
#   streamlit run app.py
#
# CSV 위치:
#   ./data 폴더 안에 .csv 파일을 넣는다.
#
# 주의:
#   avg_speed는 실제 물리 속도 m/s가 아니다.
#   avg_speed = (left_cmd + right_cmd) / 2 로 계산한
#   좌우 모터 명령값의 평균이다.

import os
import glob

import streamlit as st
import pandas as pd
import plotly.graph_objects as go


DATA_DIR = "./streamlit/data"
CENTER_POSITION = 3500


st.set_page_config(
    page_title="PAI-Car CSV Plotter",
    layout="wide"
)

st.title("PAI-Car CSV Plotter")
st.caption("`./data` 폴더 안의 CSV 파일을 선택해서 주행 데이터를 plot한다.")
st.info("반드시 `python app.py`가 아니라 `streamlit run app.py`로 실행해라.")


current_dir = os.getcwd()
data_dir_abs = os.path.abspath(DATA_DIR)

with st.expander("경로 확인", expanded=True):
    st.write("현재 작업 폴더:", current_dir)
    st.write("data 폴더 절대경로:", data_dir_abs)


if not os.path.exists(DATA_DIR):
    st.error("./data 폴더가 없습니다.")
    st.write("프로젝트 폴더 안에 data 폴더를 만들고 CSV 파일을 넣어라.")
    st.code("mkdir data", language="bash")
    st.stop()


csv_files = glob.glob(os.path.join(DATA_DIR, "*.csv"))
csv_files = sorted(csv_files, key=os.path.getmtime, reverse=True)

if len(csv_files) == 0:
    st.error("./data 폴더 안에 CSV 파일이 없습니다.")
    st.write("예시 경로:")
    st.code("./data/pai_car_run_20260709_123456.csv")
    st.stop()


csv_file_names = [os.path.basename(path) for path in csv_files]

selected_name = st.sidebar.selectbox(
    "CSV 파일 선택",
    csv_file_names
)

selected_path = os.path.join(DATA_DIR, selected_name)

st.sidebar.write("선택한 파일:")
st.sidebar.code(selected_path)


def read_csv_safely(path):
    encodings = ["utf-8-sig", "utf-8", "cp949"]

    last_error = None

    for enc in encodings:
        try:
            return pd.read_csv(path, encoding=enc), enc
        except Exception as e:
            last_error = e

    raise last_error


try:
    df, used_encoding = read_csv_safely(selected_path)
except Exception as e:
    st.error("CSV 파일을 읽지 못했습니다.")
    st.exception(e)
    st.stop()


if df.empty:
    st.warning("CSV 파일은 열렸지만 데이터가 없습니다.")
    st.stop()


df.columns = [str(col).strip() for col in df.columns]

st.success("CSV 파일을 정상적으로 읽었습니다.")
st.write("사용한 인코딩:", used_encoding)

with st.expander("CSV 컬럼 목록", expanded=True):
    st.write(list(df.columns))


numeric_columns = [
    "seq",
    "t_ms",
    "control_ms",
    "send_ms",
    "base_speed",
    "n0", "n1", "n2", "n3", "n4", "n5", "n6", "n7",
    "position",
    "error",
    "d_error",
    "left_cmd",
    "right_cmd",
    "on_line",
    "is_marker",
]

for col in numeric_columns:
    if col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")


if "t_ms" in df.columns:
    df["time_s"] = df["t_ms"] / 1000.0
else:
    st.warning("t_ms 컬럼이 없습니다. row index를 time_s 대신 사용합니다.")
    df["time_s"] = df.index.astype(float)


if "left_cmd" in df.columns and "right_cmd" in df.columns:
    df["avg_speed"] = (df["left_cmd"] + df["right_cmd"]) / 2
else:
    st.warning("left_cmd 또는 right_cmd 컬럼이 없어 avg_speed를 계산할 수 없습니다.")


if "is_marker" in df.columns:
    marker_times = df.loc[df["is_marker"] == 1, "time_s"].dropna().tolist()
else:
    marker_times = []


def safe_mean(column_name):
    if column_name not in df.columns:
        return None
    return df[column_name].mean()


row_count = len(df)

if row_count >= 2:
    run_time = df["time_s"].iloc[-1] - df["time_s"].iloc[0]
else:
    run_time = 0.0

left_avg = safe_mean("left_cmd")
right_avg = safe_mean("right_cmd")
avg_speed_mean = safe_mean("avg_speed")
marker_count = len(marker_times)


st.sidebar.header("요약")
st.sidebar.write("파일명:", selected_name)
st.sidebar.write("전체 row 수:", row_count)
st.sidebar.write("주행 시간:", "{:.3f} s".format(run_time))
st.sidebar.write("left_cmd 평균:", "{:.2f}".format(left_avg) if left_avg is not None else "없음")
st.sidebar.write("right_cmd 평균:", "{:.2f}".format(right_avg) if right_avg is not None else "없음")
st.sidebar.write("avg_speed 평균:", "{:.2f}".format(avg_speed_mean) if avg_speed_mean is not None else "없음")
st.sidebar.write("marker 감지 횟수:", marker_count)


col1, col2, col3, col4 = st.columns(4)

col1.metric("Rows", row_count)
col2.metric("Run time", "{:.3f} s".format(run_time))
col3.metric("Avg speed", "{:.2f}".format(avg_speed_mean) if avg_speed_mean is not None else "N/A")
col4.metric("Markers", marker_count)

st.warning("avg_speed는 실제 m/s 속도가 아니라 좌우 모터 명령값의 평균이다.")


def add_marker_lines(fig):
    for t in marker_times:
        fig.add_shape(
            type="line",
            x0=t,
            x1=t,
            y0=0,
            y1=1,
            xref="x",
            yref="paper",
            line=dict(
                dash="dot",
                width=1
            )
        )


def apply_layout(fig, title, y_title):
    fig.update_layout(
        title=title,
        xaxis_title="time_s",
        yaxis_title=y_title,
        hovermode="x unified",
        height=360,
        margin=dict(l=40, r=30, t=60, b=40)
    )

    fig.update_xaxes(showgrid=True)
    fig.update_yaxes(showgrid=True)


def add_horizontal_line(fig, y_value, name):
    x_min = df["time_s"].min()
    x_max = df["time_s"].max()

    fig.add_trace(
        go.Scatter(
            x=[x_min, x_max],
            y=[y_value, y_value],
            mode="lines",
            name=name,
            line=dict(
                dash="dash",
                width=1
            )
        )
    )


st.subheader("1. Position")

if "position" not in df.columns:
    st.error("position 컬럼이 없어 position 그래프를 표시할 수 없습니다.")
else:
    fig_pos = go.Figure()

    fig_pos.add_trace(
        go.Scatter(
            x=df["time_s"],
            y=df["position"],
            mode="lines",
            name="position"
        )
    )

    add_horizontal_line(fig_pos, CENTER_POSITION, "center = 3500")
    add_marker_lines(fig_pos)
    apply_layout(fig_pos, "Position vs Time", "position")

    st.plotly_chart(fig_pos, use_container_width=True)


st.subheader("2. Error")

if "error" not in df.columns:
    st.error("error 컬럼이 없어 error 그래프를 표시할 수 없습니다.")
else:
    fig_error = go.Figure()

    fig_error.add_trace(
        go.Scatter(
            x=df["time_s"],
            y=df["error"],
            mode="lines",
            name="error"
        )
    )

    add_horizontal_line(fig_error, 0, "error = 0")
    add_marker_lines(fig_error)
    apply_layout(fig_error, "Error vs Time", "error")

    st.plotly_chart(fig_error, use_container_width=True)


st.subheader("3. Motor Command / Avg Speed")

fig_motor = go.Figure()
motor_trace_count = 0

if "left_cmd" in df.columns:
    fig_motor.add_trace(
        go.Scatter(
            x=df["time_s"],
            y=df["left_cmd"],
            mode="lines",
            name="left_cmd"
        )
    )
    motor_trace_count += 1
else:
    st.warning("left_cmd 컬럼이 없습니다.")

if "right_cmd" in df.columns:
    fig_motor.add_trace(
        go.Scatter(
            x=df["time_s"],
            y=df["right_cmd"],
            mode="lines",
            name="right_cmd"
        )
    )
    motor_trace_count += 1
else:
    st.warning("right_cmd 컬럼이 없습니다.")

if "avg_speed" in df.columns:
    fig_motor.add_trace(
        go.Scatter(
            x=df["time_s"],
            y=df["avg_speed"],
            mode="lines",
            name="avg_speed"
        )
    )
    motor_trace_count += 1
else:
    st.warning("avg_speed를 계산하지 못했습니다.")

if motor_trace_count == 0:
    st.error("모터 관련 컬럼이 없어 그래프를 표시할 수 없습니다.")
else:
    add_marker_lines(fig_motor)
    apply_layout(
        fig_motor,
        "Left / Right Motor Command and Avg Speed vs Time",
        "motor command / avg_speed"
    )

    st.plotly_chart(fig_motor, use_container_width=True)


with st.expander("CSV 데이터 미리보기"):
    st.dataframe(df, use_container_width=True)