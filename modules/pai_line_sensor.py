# pai_line_sensor.py
# PAI-Car 라인센서 캘리브레이션 및 라인 위치 계산 모듈
#
# ADC:
#   - TLA2528 사용
#   - adc.read_all_raw10()은 0~1023 범위의 raw 값 8개를 반환해야 함
#
# 정규화된 출력값(norm):
#   - 흰 바닥    -> 0 근처
#   - 검은 라인 -> 1000 근처
#
# 라인 위치(position):
#   - 가장 왼쪽 센서  = 0
#   - 가장 오른쪽 센서 = 7000
#   - 중앙            = 3500

from time import ticks_ms, ticks_diff, sleep_ms

try:
    import ujson as json
except ImportError:
    import json


# ------------------------------------------------------------
# PAI-Car v1.0 제품판 기본 설정값
# ------------------------------------------------------------

# 라인으로 인정하기 위한 최소 센서 반응 합계
#
# 8개 정규화값(norm)의 합이 이 값보다 작으면
# 라인을 찾지 못한 것으로 판단한다.
#
# 제품판에서는 흰 바닥에서도 total이 200~400 정도 나올 수 있어서
# 기존 100보다 큰 값인 500을 기본값으로 사용한다.
DEFAULT_MIN_TOTAL = 500


# 약한 센서 반응을 노이즈로 보고 제거하기 위한 기준값
#
# 제품판에서는 흰 바닥에서도 각 센서의 norm(정규화된) 값이
# 40~80 정도 남는 경우가 있었다.
#
# 예:
#   norm = [65, 57, 70, 40, 58, 61, 85, 896]
#
# 이 경우 실제로는 오른쪽 끝 센서만 검은 라인을 보고 있지만,
# 나머지 센서의 작은 잔여값 때문에 position이 중앙 쪽으로 당겨질 수 있다.
#
# 그래서 위치 계산을 할 때는 NOISE_CUTOFF보다 작은 값은 0으로 처리한다.
DEFAULT_NOISE_CUTOFF = 120


class PaiLineSensor:
    def __init__(
        self,
        adc,
        sensor_count=8,
        dark_is_low=True,
        cal_file="line_cal.json",
        min_range=30,
        min_total=DEFAULT_MIN_TOTAL,
        noise_cutoff=DEFAULT_NOISE_CUTOFF
    ):
        # ADC 객체
        self.adc = adc

        # 라인센서 개수
        self.sensor_count = sensor_count

        # 검은 라인에서 raw 값이 낮아지는 센서 구조이면 True
        #
        # dark_is_low=True:
        #   raw 낮음 -> 검은 라인
        #   raw 높음 -> 흰 바닥
        #
        # dark_is_low=False:
        #   raw 높음 -> 검은 라인
        #   raw 낮음 -> 흰 바닥
        self.dark_is_low = dark_is_low

        # 캘리브레이션 데이터를 저장할 파일 이름
        self.cal_file = cal_file

        # 센서별 최소/최대값 차이가 이 값보다 작으면
        # 캘리브레이션이 충분하지 않은 것으로 판단한다.
        self.min_range = min_range

        # 라인 검출 기준값
        self.min_total = min_total

        # 위치 계산에 사용할 노이즈 제거 기준값
        self.noise_cutoff = noise_cutoff

        # 센서별 캘리브레이션 최소/최대값
        self.cal_min = [1023] * sensor_count
        self.cal_max = [0] * sensor_count

        # 라인을 잃었을 때 사용할 마지막 위치값
        self.last_position = ((sensor_count - 1) * 1000) // 2

        # 마지막 정규화값
        self.last_norm = [0] * sensor_count

        # 디버깅용 상태값
        self.last_total = 0
        self.last_filtered_total = 0
        self.last_peak = 0
        self.last_peak_index = 0

    # ------------------------------------------------------------
    # Raw 값 읽기
    # ------------------------------------------------------------

    def read_raw(self):
        """
        ADC에서 8개 라인센서 raw 값을 읽는다.

        반환값:
            raw 값 리스트, 길이 sensor_count

        오류가 발생하면 0으로 채운 리스트를 반환한다.
        """

        try:
            raw = self.adc.read_all_raw10()
        except OSError:
            # I2C/ADC 읽기 오류가 발생한 경우
            # 라인을 잃은 상황으로 처리될 수 있도록 0을 반환한다.
            return [0] * self.sensor_count

        if raw is None:
            return [0] * self.sensor_count

        # 읽어 온 값의 개수가 부족할 경우 부족한 부분은 0으로 채운다.
        if len(raw) < self.sensor_count:
            out = [0] * self.sensor_count
            for i in range(len(raw)):
                out[i] = raw[i]
            return out

        # 필요한 개수만 잘라서 반환한다.
        return raw[:self.sensor_count]

    # ------------------------------------------------------------
    # 캘리브레이션
    # ------------------------------------------------------------

    def reset_calibration(self):
        """
        캘리브레이션 최소/최대값을 초기화한다.
        """

        self.cal_min = [1023] * self.sensor_count
        self.cal_max = [0] * self.sensor_count

    def update_calibration(self, raw=None):
        """
        현재 raw 값을 이용해 센서별 최소/최대값을 갱신한다.

        캘리브레이션 중에는 PAI-Car를 검은 라인과 흰 바닥 위에서
        좌우로 움직여 모든 센서가 검은색과 흰색을 모두 보게 해야 한다.
        """

        if raw is None:
            raw = self.read_raw()

        for i in range(self.sensor_count):
            v = int(raw[i])

            # 비정상적인 0 값은 무시한다.
            # I2C/ADC 오류로 순간적으로 0이 들어오면
            # cal_min이 잘못 낮아지는 것을 막기 위한 처리이다.
            if v <= 0:
                continue

            if v < self.cal_min[i]:
                self.cal_min[i] = v

            if v > self.cal_max[i]:
                self.cal_max[i] = v

    def calibrate_for(self, duration_ms=5000, interval_ms=5, print_interval_ms=500):
        """
        지정한 시간 동안 센서값 범위를 측정한다.

        duration_ms:
            캘리브레이션 시간

        interval_ms:
            센서값을 읽는 간격

        print_interval_ms:
            중간 결과를 출력하는 간격
        """

        self.reset_calibration()

        start = ticks_ms()
        last_print = start

        while ticks_diff(ticks_ms(), start) < duration_ms:
            raw = self.read_raw()
            self.update_calibration(raw)

            now = ticks_ms()
            if ticks_diff(now, last_print) >= print_interval_ms:
                print("raw:", raw)
                print("min:", self.cal_min)
                print("max:", self.cal_max)
                print()
                last_print = now

            sleep_ms(interval_ms)

        return self.is_calibrated()

    def is_calibrated(self):
        """
        캘리브레이션 결과가 유효한지 확인한다.

        각 센서에 대해:
            cal_min < cal_max
            cal_max - cal_min >= min_range

        조건을 만족해야 한다.
        """

        for i in range(self.sensor_count):
            if self.cal_min[i] >= self.cal_max[i]:
                return False

            if self.cal_max[i] - self.cal_min[i] < self.min_range:
                return False

        return True

    # ------------------------------------------------------------
    # 캘리브레이션 저장 / 불러오기
    # ------------------------------------------------------------

    def save_calibration(self):
        """
        캘리브레이션 결과를 JSON 파일로 저장한다.
        """

        data = {
            "sensor_count": self.sensor_count,
            "dark_is_low": self.dark_is_low,
            "cal_min": self.cal_min,
            "cal_max": self.cal_max,
        }

        with open(self.cal_file, "w") as f:
            json.dump(data, f)

        print("Calibration saved:", self.cal_file)

    def load_calibration(self):
        """
        JSON 파일에서 캘리브레이션 결과를 불러온다.
        """

        with open(self.cal_file, "r") as f:
            data = json.load(f)

        self.cal_min = data["cal_min"]
        self.cal_max = data["cal_max"]

        if "dark_is_low" in data:
            self.dark_is_low = data["dark_is_low"]

        print("Calibration loaded:", self.cal_file)
        print("min:", self.cal_min)
        print("max:", self.cal_max)

    # ------------------------------------------------------------
    # 라인 검출 기준값 설정
    # ------------------------------------------------------------

    def set_line_thresholds(self, min_total=None, noise_cutoff=None):
        """
        객체 생성 후 라인 검출 기준값을 변경한다.

        예:
            line.set_line_thresholds(min_total=500, noise_cutoff=100)
        """

        if min_total is not None:
            self.min_total = int(min_total)

        if noise_cutoff is not None:
            self.noise_cutoff = int(noise_cutoff)

    # ------------------------------------------------------------
    # 정규화
    # ------------------------------------------------------------

    def normalize(self, raw=None):
        """
        raw 값을 0~1000 범위의 정규화값으로 변환한다.

        정규화 결과:
            흰 바닥    -> 0 근처
            검은 라인 -> 1000 근처

        반환값:
            norm 리스트
        """

        if raw is None:
            raw = self.read_raw()

        norm = [0] * self.sensor_count

        for i in range(self.sensor_count):
            v = int(raw[i])
            lo = self.cal_min[i]
            hi = self.cal_max[i]
            span = hi - lo

            if span < self.min_range:
                # 캘리브레이션 범위가 너무 작으면
                # 신뢰할 수 없는 센서로 보고 0 처리한다.
                value = 0
            else:
                if self.dark_is_low:
                    # 검은 라인에서 raw 값이 낮아지는 경우
                    #
                    # raw 낮음 -> 검은 라인 -> norm 1000
                    # raw 높음 -> 흰 바닥   -> norm 0
                    value = ((hi - v) * 1000) // span
                else:
                    # 검은 라인에서 raw 값이 높아지는 경우
                    #
                    # raw 높음 -> 검은 라인 -> norm 1000
                    # raw 낮음 -> 흰 바닥   -> norm 0
                    value = ((v - lo) * 1000) // span

                # 0~1000 범위를 벗어나지 않도록 제한한다.
                if value < 0:
                    value = 0
                elif value > 1000:
                    value = 1000

            norm[i] = value

        self.last_norm = norm
        return norm

    # ------------------------------------------------------------
    # 라인 위치 계산
    # ------------------------------------------------------------

    def read_line(self, min_total=None, noise_cutoff=None):
        """
        라인 위치를 계산한다.

        반환값:
            position, norm, on_line

        position:
            라인의 위치
            왼쪽 끝  = 0
            중앙     = 3500
            오른쪽 끝 = 7000

        norm:
            8개 센서의 정규화값
            디버깅을 위해 원래 정규화값을 그대로 반환한다.
            즉, noise_cutoff가 적용되지 않은 값이다.

        on_line:
            라인을 찾았으면 True
            라인을 잃었으면 False
        """

        if min_total is None:
            min_total = self.min_total

        if noise_cutoff is None:
            noise_cutoff = self.noise_cutoff

        raw = self.read_raw()
        norm = self.normalize(raw)

        total = 0
        filtered_total = 0
        weighted_sum = 0

        peak = 0
        peak_index = 0

        for i in range(self.sensor_count):
            original_v = int(norm[i])

            # 원래 norm 합계
            # 디버깅용으로 사용한다.
            total += original_v

            # 가장 강하게 반응한 센서값과 채널 번호
            if original_v > peak:
                peak = original_v
                peak_index = i

            # 흰 바닥에서 남는 약한 잔여값은
            # 위치 계산에서는 0으로 처리한다.
            if original_v < noise_cutoff:
                v = 0
            else:
                v = original_v

            filtered_total += v
            weighted_sum += i * 1000 * v

        # 디버깅용 상태값 저장
        self.last_total = total
        self.last_filtered_total = filtered_total
        self.last_peak = peak
        self.last_peak_index = peak_index

        if filtered_total < min_total:
            # 라인을 찾지 못한 경우
            # 제어 루프의 급격한 변화 방지를 위해 이전 position을 반환한다.
            return self.last_position, norm, False

        position = weighted_sum // filtered_total
        self.last_position = position

        return position, norm, True

    def read_error(self, min_total=None, noise_cutoff=None):
        """
        중앙 기준 오차를 계산한다.

        error:
            position - center

        의미:
            error < 0 -> 라인이 왼쪽에 있음
            error = 0 -> 라인이 중앙에 있음
            error > 0 -> 라인이 오른쪽에 있음
        """

        position, norm, on_line = self.read_line(
            min_total=min_total,
            noise_cutoff=noise_cutoff
        )

        center = ((self.sensor_count - 1) * 1000) // 2
        error = position - center

        return error, position, norm, on_line