import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Define the values for baud rates, packet sizes, and error rates (BCC1 + BCC2)
baud_rates = [600, 1200, 1800, 2400, 4800, 9600, 19200, 38400]
packet_sizes = [20, 40, 60, 80, 100, 120, 140, 160, 180, 200]
error_percentages = [(0, 0), (2, 2), (4, 4), (6, 6), (8, 8), (10, 10)]

# Sample efficiency calculation function (replace with real data or formula if needed)
def calculate_efficiency(packet_size, baud_rate, error_percent_bcc1, error_percent_bcc2):
    Tprop = 0.001  # Propagation delay in seconds
    Tproc = 0.001  # Processing delay in seconds
    L = packet_size
    R = baud_rate
    Tf = L / R
    efficiency = Tf / (Tf + Tprop + Tproc)
    
    # Adjust efficiency based on errors (simulated impact here for illustration)
    error_impact = 1 - (error_percent_bcc1 + error_percent_bcc2) / 200  # Simplified error impact model
    efficiency *= error_impact
    return efficiency

# Collect data for plotting
data = []

for error_bcc1, error_bcc2 in error_percentages:
    for packet_size in packet_sizes:
        for baud_rate in baud_rates:
            efficiency = calculate_efficiency(packet_size, baud_rate, error_bcc1, error_bcc2)
            data.append({
                "Packet Size": packet_size,
                "Baud Rate": baud_rate,
                "Error BCC1": error_bcc1,
                "Error BCC2": error_bcc2,
                "Efficiency": efficiency
            })

# Convert to DataFrame for easier plotting
df = pd.DataFrame(data)

# Plot efficiency by baud rate for each error rate
for error_bcc1, error_bcc2 in error_percentages:
    plt.figure(figsize=(10, 6))
    for packet_size in packet_sizes:
        subset = df[(df["Packet Size"] == packet_size) &
                    (df["Error BCC1"] == error_bcc1) &
                    (df["Error BCC2"] == error_bcc2)]
        
        plt.plot(subset["Baud Rate"], subset["Efficiency"], marker='o', label=f"Packet Size {packet_size} bytes")
    
    plt.title(f"Efficiency vs Baud Rate (Error {error_bcc1}% + {error_bcc2}%)")
    plt.xlabel("Baud Rate (bps)")
    plt.ylabel("Efficiency")
    plt.legend()
    plt.grid()
    plt.savefig(f"efficiency_vs_baud_rate_error_{error_bcc1}_{error_bcc2}.png")

# Plot efficiency by packet size for each error rate
for error_bcc1, error_bcc2 in error_percentages:
    plt.figure(figsize=(10, 6))
    for baud_rate in baud_rates:
        subset = df[(df["Baud Rate"] == baud_rate) &
                    (df["Error BCC1"] == error_bcc1) &
                    (df["Error BCC2"] == error_bcc2)]
        
        plt.plot(subset["Packet Size"], subset["Efficiency"], marker='o', label=f"Baud Rate {baud_rate} bps")
    
    plt.title(f"Efficiency vs Packet Size (Error {error_bcc1}% + {error_bcc2}%)")
    plt.xlabel("Packet Size (bytes)")
    plt.ylabel("Efficiency")
    plt.legend()
    plt.grid()
    plt.savefig(f"efficiency_vs_packet_size_error_{error_bcc1}_{error_bcc2}.png")

print("Graphs have been saved as PNG files.")
