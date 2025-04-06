import math
import os
import threading
import time
import numpy as np
import ZLAC8015D
import D_MNSV7_X16
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

# ===== Thông số xe =====
wheelbase = 25.5  # cm, Khoảng cách 2 bánh xe
Rw = 6.5  # cm, Bán kính bánh xe
t1 = 0.8  # Thời gian tăng tốc để quay góc (s)
t2 = 0.8  # Thời gian giảm tốc để quay góc (s)

# ===== Cấu hình PID =====
pid_table = {   # Thông số PID chính dùng cho tín hiệu analog (rpm: (Kp, Ki, Kd))
 0: (1.082, 0, 13.3571),
 5: (1.0406, 0, 12.7857),
 10: (0.9992, 0, 12.2143),
 15: (0.9578, 0, 11.6429),
 20: (0.9164, 0, 11.0714),
 25: (0.875, 0.0066, 10.5), # Không biết có nên cho Ki = 0 ko?
 30: (0.8336, 0.0095, 9.9286),
 35: (0.7922, 0.0129, 9.3571),
 40: (0.7508, 0.0169, 8.7857),
 45: (0.7094, 0.0214, 8.2143),
 50: (0.668, 0.0264, 7.6429),
 55: (0.6266, 0.0319, 7.0714),
 60: (0.5852, 0.038, 6.5)
}
Kp, Ki, Kd = pid_table.get(0)   # Lấy trước PID từ 0 rpm
Kp_d, Kd_d = 0.2926, 7.5   # Thông số PD riêng dùng cho tín hiệu digital, ko cần I do tín hiệu ít khi có steady-state error
target = 7.5  # Vị trí trung tâm cảm biến cho tín hiệu analog
position_analog = 0  # Giá trị vị trí cho tính toán PID analog
position_digital = 0 # Giá trị vị trí cho tính toán PID digital
error, prevError = 0.0, 0.0
d_value, i_value = 0.0, 0.0 # Giá trị đạo hàm (* Kd) và giá trị tích phân (* Ki)
correction = 0.0 # Điều chỉnh PID

# ===== Biến xử lý cho PID controller =====
cmds = [0, 0]  # Tốc độ điều khiển: Right rpm = cmds[0] (+), Left rpm = cmds[1] (-)
robot_running_analog = True  # Điều khiển trạng thái robot tín hiệu analog
robot_running_digital = False # Điều khiển trạng thái robot tín hiệu digital
pid_interval = 0.09  # Thời gian delay điều khiển PID (0.1 -> 0.09s, do trễ từ giao diện => giảm thời gian thi hành)
lost_line_timer = None # Biến lưu timer xử lý khi xe đi khỏi line
timeout_duration = 0.5  # Thời gian chờ để dừng xe khi trật khỏi line
current_speed_zone = 25 # Biến lưu vùng tốc độ cần đạt ( 60 or 25 rpm )
target_speed = current_speed_zone # Biến lưu tốc độ xe cần đạt ( điều khiển tăng tốc / giảm tốc)
# target_speed_reached = False # Biến lưu trạng thái thay đổi tốc độ
sensor_positions = [-7.5, -6.5, -5.5, -4.5, -3.5, -2.5, -1.5, -0.5,
                     0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5]  # Dùng cho điều khiển PID bằng tín hiệu digital
out_of_line = False

# ===== Biến xử lý cho nhận diện marker =====
line_pin_count = 0 # Dùng cho nhận biết ngã rẽ
digital_value = 0 # Dùng cho nhận biết ngã rẽ
marker_in_zone = False # Khi PID đo được sẽ ko tính toán sai số, cần cập nhật lại prevError giúp tránh giật
marker_check_count = 0 # marker "R" ( > 3 ) ,marker "L" ( < -13 ), tăng độ chính xác cho việc đọc marker bằng cách thêm giới hạn
intersection_marker_check_count = 0 # Tương tự marker_check_count nhưng dùng cho intersection ( > 2 )
intersection_marker_count_met = 0   # Đếm số lần đọc được marker ngã rẽ, max: 2 ( 1-in, 2-out )
total_intersection_passed = 0   # Đếm tổng số ngã rẽ đi qua, dùng để xử lý định hướng
current_analog_array, previous_analog_array = [], []
current_digital_value, prev_digital_value = 0b0, 0b0

# ===== Khóa đa luồng =====
lock_sensor = threading.Lock()  # Để tránh xung đột giữa các thread lấy dữ liệu cảm biến
lock_motors = threading.Lock()  # Để tránh xung đột giữa các thread lấy dữ liệu từ motor

# ===== Cấu hình giao tiếp Modbus =====
client1 = ModbusClient(method='rtu', port="COM10", baudrate=115200, timeout=1)
client2 = ModbusClient(method='rtu', port="COM3", baudrate=115200, timeout=1)

# ===== Khởi tạo động cơ và cảm biến =====
motors = ZLAC8015D.Controller(client1)
sensor = D_MNSV7_X16.Magnetic_Line_Sensor(client2)

## =================== HÀM DỪNG KHẨN CẤP AGV ===================
# Note: Khi gặp vật cản hoặc ra khỏi line
def AGV_emergency_stop():
    global cmds, robot_running_analog, robot_running_digital, lost_line_timer, out_of_line
    global Kp, Ki, Kd
    # global target_speed_reached
    if robot_running_analog or robot_running_digital:
        print("Dừng khẩn cấp xe!")
        robot_running_analog = False
        robot_running_digital = False
        with lock_motors:
            left_rpm, right_rpm = motors.get_rpm() # Do vị trí bánh đảo ngược nên left_rpm là của bánh phải (right) và right_rpm là của bánh trái (left)
            average_rpm = (abs(left_rpm) + abs(right_rpm)) / 2
            stop_time = int((average_rpm / 60) * 1500) # ms, Tiêu chuẩn thời gian dừng cho 60 rpm là 1.5s (1500 ms)
            motors.set_decel_time(stop_time, stop_time)
            motors.stop()
        time.sleep(stop_time/1000 + 0.5)   # Đảm bảo điều khiển PID đã được tắt
        cmds = [0, 0]  # Reset lại tốc độ ban đầu
        Kp, Ki, Kd = pid_table.get(0)   # Cập nhật PID về ban đầu
        # target_speed_reached = False
        print("Xe đã dừng!")
        motors.enable_motor() # Xe cần khởi động lại sau khi dừng và cần vô hiệu hóa bánh xe để thả tự do
        motors.set_accel_time(50, 50)
        motors.set_decel_time(50, 50)
        print("Khởi động lại xe...")
        time.sleep(0.5)

        # plot_error()
        # motors.disable_motor()

        if lost_line_timer is not None:
            lost_line_timer.cancel()
            lost_line_timer = None
            motors.disable_motor()  # Thả tự do bánh xe khi đã rời khỏi line
            print("Vô hiệu hóa bánh xe!")
            out_of_line = True

        print(f"Marker count: {marker_check_count}")
        print(f"Intersection marker count: {intersection_marker_check_count}")
    # time.sleep(1)
    # turn_angle(90, 'L')

## =================== HÀM DỪNG AGV VỚI BIẾN THỜI GIAN ===================
# Note: Khi cần dừng với thời gian cần thiết (tính toán quãng đường), dùng cho ngã rẽ
def AGV_stop_with_time(stop_time):
    global robot_running_analog, robot_running_digital, cmds
    global Kp, Ki, Kd
    # global target_speed_reached
    if robot_running_analog or robot_running_digital:
        print("Dừng xe theo thời gian!")
        robot_running_analog = False
        robot_running_digital = False
        with lock_motors:
            motors.set_decel_time(stop_time, stop_time)
            motors.stop()
        time.sleep(stop_time/1000 + 0.5)   # Đảm bảo điều khiển PID đã được tắt
        cmds = [0, 0]  # Reset lại tốc độ ban đầu
        Kp, Ki, Kd = pid_table.get(0)  # Cập nhật PID về ban đầu
        # target_speed_reached = False
        print("Xe đã dừng!")
        motors.enable_motor()  # Xe cần khởi động lại sau khi dừng, kết hợp với turn_angle()
        # motors.set_accel_time(50, 50)
        # motors.set_decel_time(50, 50)
        print("Khởi động lại xe...")
        # time.sleep(0.5)

# =================== HÀM KIỂM TRA LIỆU CÓ DUY NHẤT TÍN HIỆU ANALOG TỪ 1 LINE TỪ ===================
def has_only_1_surge_signal(arr):
    len_arr = len(arr)
    peak_index = np.argmax(arr)

    # Check increasing sequence before the peak
    for i in range(peak_index):
        if arr[i] > arr[i + 1]:
            return False

    # Check decreasing sequence after the peak
    for i in range(peak_index, len_arr - 1):
        if arr[i] < arr[i + 1]:
            return False

    return True
# Tương tự hàm trên nhưng dùng đặc biệt cho marker
def has_only_1_surge_signal_marker(arr):
    len_arr = len(arr)
    peak_index = np.argmax(arr)

    # Check increasing sequence before the peak
    for i in range(peak_index):
        if arr[i] > arr[i + 1]:
            return False, arr[peak_index]

    # Check decreasing sequence after the peak
    for i in range(peak_index, len_arr - 1):
        if arr[i] < arr[i + 1]:
            return False, arr[peak_index]

    return True, arr[peak_index]

# =================== HÀM TÍNH GIÁ TRỊ VỊ TRÍ CỦA CẢM BIẾN LINE TỪ THEO TÍN HIỆU ANALOG ===================
def get_position_value_analog(pin_values):
    # Calculate the weighted sum and total sum of sensor values
    weighted_sum = sum(i * value for i, value in enumerate(pin_values))
    total_sum = sum(pin_values)

    # Avoid division by zero if no sensor is activated
    if total_sum == 0:
        return None  # No line detected

    # Calculate the position as a weighted average
    position_value = weighted_sum / total_sum
    return position_value

# =================== HÀM ĐIỀU KHIỂN PID TÍN HIỆU ANALOG ===================
def pid_controller_analog():
    global cmds, position_analog, error, prevError, d_value, i_value, correction, lost_line_timer
    global Kp, Ki, Kd
    global marker_in_zone, target_speed
    print("Start PID analog")
    while robot_running_analog:
        start = time.perf_counter()

        # # Tăng/giảm tốc độ từ từ
        # if not target_speed_reached:
        #     if cmds[0] == target_speed:
        #         target_speed_reached = True
        #     else:
        #         delta = 2.5 if cmds[0] < target_speed else -2.5
        #         cmds[0] += delta
        #         cmds[1] -= delta
        #         Kp, Ki, Kd = pid_table.get(cmds[0], (Kp, Ki, Kd))

        # Tăng/giảm tốc độ từ từ
        if cmds[0] != target_speed:
            delta = 2.5 if cmds[0] < target_speed else -2.5
            cmds[0] += delta
            cmds[1] -= delta
            Kp, Ki, Kd = pid_table.get(cmds[0], (Kp, Ki, Kd))

        # Chờ lock_sensor được thả từ marker_check()
        with lock_sensor:
            analog_array = sensor.get_analog_output()

        # Chỉ bám line khi tín hiệu là từ 1 line từ duy nhất
        if has_only_1_surge_signal(analog_array):
            position_analog = get_position_value_analog(analog_array)
            if position_analog is None:
                print("Không tìm thấy line!")
                if lost_line_timer is None:
                    print("Tiến hành dò tìm line...")
                    lost_line_timer = threading.Timer(timeout_duration, AGV_emergency_stop)
                    lost_line_timer.start()
                correction = 0
            else:
                # Đã dò được line, dừng timer xử lý mất line
                if lost_line_timer is not None:
                    print("Đã dò được line, tiếp tục điều hướng!")
                    lost_line_timer.cancel()
                    lost_line_timer = None

                # Cần cập nhật biến prevError ngay khi ra khỏi marker để tránh giật
                if marker_in_zone:
                    prevError = get_position_value_analog(sensor.get_analog_output()) - target
                    marker_in_zone = False

                # Tính toán PID
                error = position_analog - target
                d_value = error - prevError
                i_value += error
                correction = Kp * error + Kd * d_value + Ki * i_value

                # Giới hạn lại độ điều chỉnh tránh xe lắc mạnh
                if correction > 20: correction = 20
                elif correction < -20: correction = -20

                # Cập nhật biến lỗi quá khứ
                prevError = error
        else:
            marker_in_zone = True
            correction = 0

        # Điều chỉnh tốc độ động cơ
        left_speed = int(cmds[1] - correction)
        right_speed = int(cmds[0] - correction)
        # Can thiệp khi cần thiết (AGV_emergency_stop/AGV_stop_with_time)
        with lock_motors:
            motors.set_rpm(right_speed, left_speed)

        end = time.perf_counter()
        print(end-start)

        # Thời gian delay để tránh quá tải, race giữa các thread
        time.sleep(pid_interval)

        # Dừng khi hoàn thành quãng đường, giảm dần về 0 để bám line
        if destination_reached and cmds[0] == 0:
            with lock_motors:
                motors.stop()
                time.sleep(0.5)
                Kp, Ki, Kd = pid_table.get(0)  # Cập nhật PID về ban đầu
                motors.enable_motor()
                turn_angle(180, 15, 'L', 2) # Quay 180 độ
                target_speed = current_speed_zone # Cập nhật lại tốc độ cần đạt
                break
    print("PID analog has stopped")

# =================== HÀM CHUYỂN GIÁ TRỊ HEX VỀ 1 MẢNG 16 PHẦN TỬ SỐ NGUYÊN ===================
def read_sensor(hex_value):
    binary_string = format(hex_value, '016b')  # Chuyển hex sang binary
    return [int(bit) for bit in binary_string]  # Trả về 1 mảng 16 phần tử gồm 0s và 1s

# =================== HÀM TÍNH GIÁ TRỊ VỊ TRÍ CỦA CẢM BIẾN LINE TỪ THEO TÍN HIỆU DIGITAL ===================
def get_position_value_digital():
    weighted_sum = 0
    active_sensors = 0

    with lock_sensor:  # Chờ lock_sensor được thả từ marker_check()
        sensor_digital = sensor.get_digital_output()

    sensor_digital_data = read_sensor(sensor_digital)
    for i, bit in enumerate(sensor_digital_data):
        if bit == 1:
            weighted_sum += sensor_positions[i]
            active_sensors += 1

    # Avoid division by zero if no sensor is activated
    if active_sensors == 0:
        return None, 0  # No line detected

    return weighted_sum / active_sensors, active_sensors

# =================== HÀM ĐIỀU KHIỂN PID TÍN HIỆU DIGITAL ===================
def pid_controller_digital():
    global cmds, position_digital, error, prevError, d_value, correction, lost_line_timer
    # global target_speed_reached
    print("Start PID digital")
    while robot_running_digital:
        position_digital, pin_count = get_position_value_digital()

        # # Tăng/giảm tốc độ từ từ
        # if not target_speed_reached:
        #     if cmds[0] == target_speed:
        #         target_speed_reached = True
        #     else:
        #         delta = 2.5 if cmds[0] < target_speed else -2.5
        #         cmds[0] += delta
        #         cmds[1] -= delta

        # Tăng/giảm tốc độ từ từ
        if cmds[0] != target_speed:
            delta = 2.5 if cmds[0] < target_speed else -2.5
            cmds[0] += delta
            cmds[1] -= delta

        if position_digital is None:
            print("Không tìm thấy line!")
            if lost_line_timer is None:
                print("Tiến hành dò tìm line...")
                lost_line_timer = threading.Timer(timeout_duration, AGV_emergency_stop)
                lost_line_timer.start()
            correction = 0
        else:
            # Đã dò được line, dừng timer xử lý mất line
            if lost_line_timer is not None:
                print("Đã dò được line, tiếp tục điều hướng!")
                lost_line_timer.cancel()
                lost_line_timer = None

            # Tránh việc đo được marker dẫn đến xe dao động mạnh
            if pin_count < 7:

                # Tính toán PID
                error = position_digital
                d_value = error - prevError

                # Giới hạn khoảng điều khiển
                if abs(error) < 1.0:
                    correction = 0
                else:
                    correction = Kp_d * error + Kd_d * d_value

                # Giới hạn lại độ điều chỉnh tránh xe lắc mạnh
                if correction > 20: correction = 20
                elif correction < -20: correction = -20

                # Cập nhật biến lỗi quá khứ
                prevError = error
            else:
                correction = 0

        # Điều chỉnh tốc độ động cơ
        left_speed = int(cmds[1] - correction)
        right_speed = int(cmds[0] - correction)
        # Can thiệp khi cần thiết (AGV_emergency_stop)
        with lock_motors:
            motors.set_rpm(right_speed, left_speed)

        # Thời gian delay để tránh quá tải, race giữa các thread
        time.sleep(pid_interval)
    print("PID digital has stopped")

# =================== HÀM PHÁT HIỆN MARKER TỪ CẢM BIẾN detect_marker_shift ===================
# Note: Dùng chung với hàm has_only_1_surge_signal_marker() để xác định marker đúng hơn
def detect_marker_shift(prev, curr):
    # Find the weighted center of mass of the previous and current distributions
    prev_center = np.average(np.arange(len(prev)), weights=prev)
    curr_center = np.average(np.arange(len(curr)), weights=curr)

    # Compare center shifts
    if curr_center < prev_center:
        return "L"
    elif curr_center > prev_center:
        return "R"
    else:
        return "C"

# =================== HÀM KIỂM TRA VẠCH BÁO RẼ Ở NGÃ BA, NGÃ TƯ detect_intersection_marker() ===================
def count_ones(value):
    return bin(value).count('1')
def compute_center_of_mass(value):
    """Calculate the average position of '1's (center of mass)."""
    binary_str = format(value, '016b')
    positions = [i for i, bit in enumerate(binary_str) if bit == '1']
    len_pos = len(positions)
    if len_pos == 0: return None
    return sum(positions) / len_pos
def detect_intersection_marker(prev, curr):
    prev_count = count_ones(prev)
    curr_count = count_ones(curr)
    # Cách biệt số lượng bit 1 quá bé, có thể loại bỏ
    if curr_count - prev_count <= 1:
        return False

    prev_com = compute_center_of_mass(prev)
    curr_com = compute_center_of_mass(curr)
    if prev_com is None or curr_com is None:
        return False # Đa phần là do ko dò được line

    # So sánh trung tâm vị trí bit 1
    shift = curr_com - prev_com

    if shift < -0.5 or shift > 0.5:
        return False
    return True # Vị trí trung tâm hoặc sai lệch ko đáng kể

## =================== HÀM/CHƯƠNG TRÌNH ĐO QUÃNG ĐƯỜNG CẦN ĐI ===================
# Note: Khi cần đi 1 quãng đường nhất định để thực hiện các tác vụ khác ( chỉ dùng cho ngã rẽ )
def travel_distance(direction):
    global robot_running_analog, robot_running_digital, pid_thread
    global current_speed_zone, target_speed, i_value, prevError, Kp, Ki, Kd
    global stop_turn_distance
    start_pul_r, start_pul_l = motors.get_pulses_travelled()

    if direction is not None:
        if current_speed_zone == 60: stop_turn_distance = 37
        else: stop_turn_distance = 38.5

        while True:
            with lock_motors:
                right_rpm, left_rpm = motors.get_rpm()  # Do vị trí bánh đảo ngược nên left_rpm là của bánh phải (right) và right_rpm là của bánh trái (left)
                curr_pul_r, curr_pul_l = motors.get_pulses_travelled()
                dist_travelled_r, dist_travelled_l = motors.get_wheels_travelled(start_pul_r, start_pul_l, curr_pul_r, curr_pul_l)
            average_rpm = (abs(left_rpm) + abs(right_rpm)) / 2
            stop_time = int((average_rpm / 60) * 1500)  # ms, Tiêu chuẩn thời gian dừng cho 60 rpm là 1.5s (1500 ms)
            linear_average_v = (average_rpm*2*math.pi/60.0) * Rw # cm/s
            stop_distance = linear_average_v * stop_time / 2000  # Khi xe dừng sẽ còn đi 1 quãng đường nữa theo vận tốc hiện tại
            current_distance = (dist_travelled_r + dist_travelled_l) / 2
            if current_distance + stop_distance >= stop_turn_distance:
                print("Tiến hành dừng và rẽ")
                AGV_stop_with_time(stop_time) # Trạng thái PID digital và analog đã được gán False, xe đã dừng => Cần khởi động lại PID analog sau khi rẽ
                turn_angle(90, 15, direction, 1)
                current_speed_zone = 25
                target_speed = current_speed_zone
                # target_speed_reached = False
                i_value = 0  # Reset giá trị tích phân về ban đầu giúp tránh tích tụ errors
                prevError = get_position_value_analog(sensor.get_analog_output()) - target # Giảm giật lúc bắt đầu PID
                # Kp, Ki, Kd = 0.875, 0, 10.5
                robot_running_analog = True
                pid_thread = threading.Thread(target=pid_controller_analog) # Khởi tạo luồng PID analog
                pid_thread.start()
                break
            time.sleep(0.02)
    else:
        robot_running_digital = True
        robot_running_analog = False
        pid_thread.join() # Đợi PID analog kết thúc
        pid_thread = threading.Thread(target=pid_controller_digital)  # Khởi tạo luồng PID digital
        pid_thread.start()

    while True:
        curr_pul_r, curr_pul_l = motors.get_pulses_travelled()
        dist_travelled_r, dist_travelled_l = motors.get_wheels_travelled(start_pul_r, start_pul_l, curr_pul_r, curr_pul_l)
        current_distance = (dist_travelled_r + dist_travelled_l) / 2
        if current_distance >= resume_marker_distance:
            break
        time.sleep(0.02)

# =================== HÀM CHUYỂN HƯỚNG ROBOT ===================
def turn_angle(angle, rpm, direction, active_wheels):
    """
    Turns the AGV by a given angle.
    :param angle: Turn angle in degrees (always positive)
    :param rpm: Wheel speed in RPM
    :param direction: "L" for left, "R" for right
    :param active_wheels: Number of wheels turning (1 or 2)
    """
    motors.set_accel_time(t1 * 1000, t1 * 1000)
    motors.set_decel_time(t2 * 1000, t2 * 1000)

    turn_radius = wheelbase / 2 if active_wheels == 2 else wheelbase  # Adjust radius based on mode
    arc_length = math.radians(angle) * turn_radius  # Distance wheel must travel

    turn_dir = 1 if direction == 'L' else -1
    if active_wheels == 2:
        motors.set_rpm(turn_dir * rpm, turn_dir * rpm)  # Counter-rotation
    elif active_wheels == 1:
        # One wheel moves, the other stays still
        right_rpm, left_rpm = (turn_dir * rpm, 0) if direction == 'L' else (0, turn_dir * rpm)
        motors.set_rpm(right_rpm, left_rpm)
    else:
        raise ValueError("Number of active wheels must be 1 or 2")

    # Get initial travel pulses
    start_pul_r, start_pul_l = motors.get_pulses_travelled()
    while True:
        curr_pul_r, curr_pul_l = motors.get_pulses_travelled()
        dist_travelled_r, dist_travelled_l = motors.get_wheels_travelled(start_pul_r, start_pul_l, curr_pul_r, curr_pul_l)
        right_rpm, left_rpm = motors.get_rpm()
        stop_distance_r = abs(right_rpm) * math.pi * Rw * t2 / 60
        stop_distance_l = abs(left_rpm) * math.pi * Rw * t2 / 60
        if active_wheels == 2:
            if dist_travelled_r + stop_distance_r >= abs(arc_length) and \
                dist_travelled_l + stop_distance_l >= abs(arc_length):
                break
        else:  # One-wheel pivot turn
            moving_dist = dist_travelled_r if direction == 'L' else dist_travelled_l
            stop_distance = stop_distance_r if direction == 'L' else stop_distance_l

            if moving_dist + stop_distance >= abs(arc_length):
                break

    motors.stop()  # Stop with deceleration time
    time.sleep(t2 + 0.5)
    motors.enable_motor()
    motors.set_accel_time(50, 50)
    motors.set_decel_time(50, 50)
    print("Khởi động lại xe sau khi rẽ...")
    time.sleep(0.5)

# =================== HÀM KIỂM TRA NGÃ RẼ ===================
def is_intersection_marker(intersect_marker_check_count):
    if intersect_marker_check_count < 3: return False

    if intersect_marker_check_count > 2 and current_speed_zone == 60:
        return True
    if intersect_marker_check_count > 8 and current_speed_zone == 25:
        return True

    return False

# =================== HÀM XỬ LÝ ĐỊNH HƯỚNG CHO NGÃ RẼ ===================
def handle_intersection(intersection_count):
    global robot_running_analog, robot_running_digital, pid_thread, target_speed
    global start_passed, destination_reached, total_intersection_passed

    if not start_passed:
        total_intersection_passed -= 1
        start_passed = True
        return
    if total_intersection_passed > total_intersection_required:
        total_intersection_passed = 0
        target_speed = 0
        # target_speed_reached = False
        destination_reached = True
        return

    direction = directions.get(intersection_count, None)
    distance_thread = threading.Thread(target=travel_distance, args=(direction,))
    distance_thread.start()

    distance_thread.join()

# =================== CHƯƠNG TRÌNH KIẾM TRA MARKER TỔNG THỂ ===================
def marker_check():
    global Kp, Ki, Kd, i_value, prevError, current_speed_zone, target_speed, robot_running_analog, robot_running_digital, pid_thread
    global line_pin_count, digital_value
    global marker_check_count, intersection_marker_check_count, intersection_marker_count_met, total_intersection_passed
    global current_analog_array, previous_analog_array
    global current_digital_value, prev_digital_value
    while not out_of_line or not destination_reached:
        # start = time.perf_counter()
        try:
            with lock_sensor:
                analog_array = sensor.get_analog_output()
            check_line_condition, max_value = has_only_1_surge_signal_marker(analog_array)
            if check_line_condition:
                previous_analog_array = analog_array

                # Thay đổi tốc độ
                if marker_check_count < -13 and current_speed_zone == 25: # Tăng tốc
                    current_speed_zone = 60
                    target_speed = current_speed_zone
                    # target_speed_reached = False
                    i_value = 0 # Reset giá trị tích phân về ban đầu giúp tránh tích tụ errors

                if marker_check_count > 3 and current_speed_zone == 60:   # Giảm tốc
                    current_speed_zone = 25
                    target_speed = current_speed_zone
                    # target_speed_reached = False
                    i_value = 0 # Reset giá trị tích phân về ban đầu giúp tránh tích tụ errors

                marker_check_count = 0 # Reset lại biến đếm marker

                with lock_sensor:
                    # Nhận diện vạch báo rẽ
                    line_pin_count, digital_value = sensor.get_pin_count()
                if line_pin_count in [5, 6]:
                    prev_digital_value = digital_value
                    if is_intersection_marker(intersection_marker_check_count): # Đã nhận diện được vạch báo ngã rẽ
                        intersection_marker_count_met += 1
                        if intersection_marker_count_met >= 2:
                            if robot_running_digital:
                                robot_running_analog = True
                                robot_running_digital = False
                                pid_thread.join() # Đợi PID digital kết thúc
                                pid_thread = threading.Thread(target=pid_controller_analog)  # Khởi tạo luồng PID analog
                                pid_thread.start()
                            elif robot_running_analog:
                                current_speed_zone = 60
                                target_speed = current_speed_zone
                                # target_speed_reached = False
                                i_value = 0  # Reset giá trị tích phân về ban đầu giúp tránh tích tụ errors

                            intersection_marker_count_met = 0
                        else:
                            total_intersection_passed += 1
                            # print(total_intersection_passed)
                            handle_intersection(total_intersection_passed) # Xử lý định hướng

                    intersection_marker_check_count = 0 # Reset biến đếm xác nhận intersection
                elif line_pin_count > 6 and max_value > 50:
                    current_digital_value = digital_value
                    # if detect_intersection_marker(prev_digital_value, current_digital_value) and not intersection_marker_met:
                    if detect_intersection_marker(prev_digital_value, current_digital_value):
                        # intersection_marker_met = True
                        intersection_marker_check_count += 1
                        print("Intersection marker detected")

            elif max_value > 20: # Tránh nhiễu hoặc line quá xa tâm
                current_analog_array = analog_array
                marker_position = detect_marker_shift(previous_analog_array, current_analog_array)
                # print(f"Current_analog = {current_analog_array}")
                # print("Marker spotted!")

                if marker_position == 'L':
                    # Left marker: Increase speed and change to default configuration
                    print("Left marker detected, minus to marker_count")
                    marker_check_count -= 1 # Update current state
                elif marker_position == 'R':
                    # Right marker: Decrease speed and change to configuration for slower speed
                    # Use this when entering turning track
                    print("Right marker detected, decreasing speed")
                    marker_check_count += 1 # Update current state
                else:
                    print("Center marker detected, AGV oscillates too much")

        except KeyboardInterrupt:
            break
        time.sleep(0.02)
        # end = time.perf_counter()
        # print(end-start)

# =================== ĐỌC FILE DẪN ĐƯỜNG ===================
def read_direction_file(base_path, start_num, end_num):
    directory_path = os.path.join(base_path, str(start_num))
    file_path = os.path.join(directory_path, f"{start_num}_{end_num}.txt")

    if os.path.exists(file_path):
        with open(file_path, 'r', encoding='utf-8') as file:
            lines = file.readlines()
            num_intersections = int(lines[0].strip())
            num_directions = eval(lines[1].strip()) if len(lines) > 1 else {}
    else:
        print("File not found.")
    return num_intersections, num_directions

# Thư mục file dẫn đường
directory = "E:\Food_Delivery_System\Robot_Map_Directions_2"

# Các biến xử lý quãng đường đi (centimeter)
stop_turn_distance = 37     # Quãng đường cần dừng để xoay bánh 'L'/'R'
resume_marker_distance = 80 # Quãng đường tổng thể cần đi để tiếp tục chương trình marker_check()

# Điểm bắt đầu và các điểm đến
start_table = 0
end_tables = [3, 4, 1, 2] # Minh họa điểm đến xếp theo thứ tự

# ===== Kích hoạt động cơ =====
motors.disable_motor()
motors.set_mode(3)
motors.enable_motor()
motors.set_accel_time(50, 50)
motors.set_decel_time(50, 50)

# ===== KÍCH HOẠT CHƯƠNG TRÌNH AGV GIAO ĐỒ ĂN TRONG NHÀ HÀNG =====
num_destination = len(end_tables)
for index in range(num_destination + 1):
    if index == 0:
        total_intersection_required, directions = read_direction_file(directory, start_table, end_tables[index])
    elif index == num_destination:
        total_intersection_required, directions = read_direction_file(directory, end_tables[index-1], start_table)
    else:
        total_intersection_required, directions = read_direction_file(directory, end_tables[index-1], end_tables[index])

    # Khởi động biến trạng thái AGV
    robot_running_analog = True  # Mặc định bắt đầu điều khiển tín hiệu analog
    robot_running_digital = False
    current_speed_zone = 25
    target_speed = current_speed_zone
    # target_speed_reached = False  # Biến lưu trạng thái thay đổi tốc độ
    out_of_line = False

    # Biến xác nhận điểm bắt đầu và điểm đến
    start_passed = False
    destination_reached = False

    # =================== KHỞI TẠO CÁC THREAD ===================
    pid_thread = threading.Thread(target=pid_controller_analog)
    marker_thread = threading.Thread(target=marker_check)

    # =================== KHỞI TẠO TÍN HIỆU ĐIỀU KHIỂN ===================
    i_value = 0  # Tránh tích tụ lỗi
    prevError = get_position_value_analog(sensor.get_analog_output()) - target  # Tránh giật xe

    # =================== BẮT ĐẦU CÁC THREAD ===================
    pid_thread.start()
    marker_thread.start()

    # =================== ĐỢI CÁC THREAD KẾT THÚC ===================
    marker_thread.join()
    pid_thread.join()

    # Chờ 3 phút để khách lấy đồ trên xe xuống và tiếp tục
    time.sleep(180)
