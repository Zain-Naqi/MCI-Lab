import serial
import csv
import time

# === Serial Port Setup ===
port = '/dev/ttyACM0'
baud_rate = 115200
ser = serial.Serial(port, baud_rate, timeout=1)

# === Signal Labels ===
signals = ["Accelerometer", "Gyroscope", "Angle"]
csv_filename = "output.csv"

# === Open CSV File for Writing ===
with open(csv_filename, mode='w', newline='') as csv_file:
    writer = csv.writer(csv_file)
    
    # Write header row: Time + signal labels
    header = ['Time (ms)'] + signals
    writer.writerow(header)

    print(f"Logging started. Writing to {csv_filename}... Press Ctrl+C to stop.\n")
    
    cnt = 0  # Millisecond time counter
    start_time = time.time()

    try:
        while True:
            line = ser.readline().decode().strip()
            values = line.split(',')

            # Skip malformed lines
            if len(values) != len(signals):
                print(f"Skipping malformed line: {line}")
                continue

            # Convert to float and write to CSV
            try:
                float_values = [float(val) for val in values]
                elapsed_ms = int((time.time() - start_time) * 1000)
                row = [elapsed_ms] + float_values
                writer.writerow(row)
                print(f"{row}")
            except ValueError:
                print(f"Skipping non-numeric line: {line}")
                continue

    except KeyboardInterrupt:
        print("\nLogging stopped by user.")
    except Exception as e:
        print("Error:", e)
