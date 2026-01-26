# analyze_timings.py
import math

INPUT_FILE = "/home/gepetto/ros2_ws/src/timingsobj_26.txt"

# load numbers
with open(INPUT_FILE, "r") as f:
    values = [float(line.strip()) for line in f if line.strip()]

if len(values) == 0:
    raise ValueError("No data found in file.")

mean = sum(values) / len(values)

variance = sum((x - mean) ** 2 for x in values) / len(values)
std = math.sqrt(variance)

print(f"Count: {len(values)}")
print(f"Mean: {mean:.6f} seconds")
print(f"Std:  {std:.6f} seconds")