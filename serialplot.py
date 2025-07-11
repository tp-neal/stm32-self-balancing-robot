import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import time

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM1'
BAUDRATE = 115200
DATA_BUFFER_SIZE = 100
Y_MIN = -100
Y_MAX = 100
X_MIN = 0
X_MAX = DATA_BUFFER_SIZE

ACCELEROMETER_KEY = 'accel-pitch'
GYROSCOPE_KEY = 'gyro-pitch'
COMPLEMENTARY_KEY = 'comp-pitch'
KEYS = [ACCELEROMETER_KEY, GYROSCOPE_KEY, COMPLEMENTARY_KEY]

# --- Global Variables ---
ser = None
NUM_PLOTS = 3
data_deques = {}
lines = {}
ax = None

def init_plot():
    for i in range(NUM_PLOTS):
        lines[KEYS[i]].set_data([], [])
    return lines.values()

def update_plot(frame):
    global ser, data_deques, lines, ax

    try:
        if not ser or not ser.is_open:
            return lines.values()

        latest_packet = ""
        # Read all data in the buffer, keeping only the last complete line.
        if ser.in_waiting > 0:
            while ser.in_waiting > 0:
                try:
                    packet = ser.readline().decode('utf-8').strip()
                    if packet:
                        latest_packet = packet
                except UnicodeDecodeError:
                    print("Warning: Serial decode error.")
                    pass
            
            # Process the single latest packet AFTER the while loop is finished.
            if latest_packet:
                tokens = latest_packet.split()
                print(f"Tokens: {tokens}")

                if len(tokens) % 2 != 0:
                    return lines.values()

                parsed_values = {}
                for i in range(0, len(tokens), 2):
                    key = tokens[i].replace(':', '')
                    try:
                        value = float(tokens[i+1])
                        if key in KEYS:
                            parsed_values[key] = value
                    except (ValueError, IndexError):
                        continue

                # Update plot data once with the most recent values.
                for key, value in parsed_values.items():
                    data_deques[key].appendleft(value)
                    lines[key].set_data(range(len(data_deques[key])), data_deques[key])

    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred in update_plot: {e}")

    return lines.values()

# --- Main Execution Block
if __name__ == "__main__":

    # Connect to serial port
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1) # KEY: timeout prevents freezing
        ser.reset_input_buffer()
        print(f"Connected to serial port: {ser.name}")
    except serial.SerialException as e:
        print(f"ERROR: Could not open serial port {SERIAL_PORT}: {e}")
        print("Make sure your device is connected and the port is correct/available.")
        exit()

    # Initiate empty deques
    for i in range(NUM_PLOTS):
        data_deques[KEYS[i]] = deque([0] * DATA_BUFFER_SIZE, maxlen=DATA_BUFFER_SIZE)

    # Configure graph
    fig, ax = plt.subplots()
    lines[ACCELEROMETER_KEY], = ax.plot(range(100), data_deques[ACCELEROMETER_KEY], label='accelerometer pitch')
    lines[GYROSCOPE_KEY], = ax.plot(range(100), data_deques[GYROSCOPE_KEY], label='gyroscope pitch')
    lines[COMPLEMENTARY_KEY], = ax.plot(range(100), data_deques[COMPLEMENTARY_KEY], label='complementary pitch')
    ax.set_title('Real-time MCU Serial Data Plot')
    ax.set_xlabel('Time (Last 100 samples)')
    ax.set_ylabel('Estimated Pitch')
    ax.grid(True)
    ax.legend()
    padding = (Y_MAX - Y_MIN) * 0.1
    ax.set_ylim(Y_MIN - padding, Y_MAX + padding)
    ax.set_xlim(X_MIN, X_MAX)

    ani = FuncAnimation(fig, update_plot, init_func=init_plot, interval=10, blit=True, cache_frame_data=False)

    plt.show()

    if ser and ser.is_open:
        ser.close()
        print("Serial port closed.")