import matplotlib.pyplot as plt
import csv

# === Configuration ===
csv_filename = "output.csv"
signals = ["Accelerometer", "Gyroscope", "Angle"]
time_ms = []
signal_data = [[] for _ in signals]  # List of lists

# === Read CSV File ===
with open(csv_filename, mode='r') as csv_file:
    reader = csv.reader(csv_file)
    header = next(reader)  # Skip header

    for row in reader:
        time_ms.append(int(row[0]))
        for i in range(len(signals)):
            signal_data[i].append(float(row[i + 1]))

# === Plotting ===
plt.figure(figsize=(10, 6))
for i, data in enumerate(signal_data):
    plt.plot(time_ms, data, label=signals[i])

plt.title("Sensor Signals from UART")
plt.xlabel("Time (ms)")
plt.ylabel("Value")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
