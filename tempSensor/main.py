# TODO: generate more data
import numpy as np

# Set random seed for reproducibility
np.random.seed(69)

# Matrix dimensions
num_sensors = 32
num_samples = 500

# Generate realistic battery temperature data with EMI noise

# Battery cell characteristics:
# - Base temperature: 25-35°C (typical Li-ion operating temp)
# - Gradual heating during discharge: +0.02°C per sample
# - Cell-to-cell variation: ±2°C (some cells heat faster)
# - EMI noise: 10V peak-to-peak random spikes

temperature_data = np.zeros((num_sensors, num_samples))

# Time array
time = np.linspace(0, num_samples-1, num_samples)

# Base temperatures for each cell (slight variation due to position in pack)
base_temps = 28 + np.random.uniform(-2, 2, num_sensors)

for sensor_id in range(num_sensors):
    # Base temperature for this cell
    base_temp = base_temps[sensor_id]
    
    # Gradual heating over time (discharge heating)
    # Cells closer to center of pack heat slightly faster
    heating_rate = 0.02 + (0.005 if 8 <= sensor_id <= 23 else 0)
    trend = heating_rate * time
    
    # Small thermal fluctuations (±0.5°C, slow variation)
    thermal_noise = np.random.normal(0, 0.5, num_samples)
    
    # EMI spikes: 10V peak-to-peak
    # Convert voltage to temperature using sensor characteristics
    # From your lookup table: ~2.4V @ -40°C to ~1.3V @ 120°C
    # Voltage range = 1.1V over 160°C span
    # 10V spike = 10V / 1.1V * 160°C ≈ 1454°C equivalent (obviously clipped)
    # But in reality, these are brief spikes that appear as large temp excursions
    
    # EMI spikes occur randomly, brief duration
    emi_noise = np.zeros(num_samples)
    
    # Generate random EMI events (5-15 per sensor over 500 samples)
    num_spikes = np.random.randint(5, 16)
    spike_locations = np.random.choice(num_samples, size=num_spikes, replace=False)
    
    for spike_loc in spike_locations:
        # EMI spike characteristics:
        # - Random amplitude: ±50°C to ±150°C (10V p-p in temp equivalent)
        # - Very brief: 1-3 samples wide
        # - Can be positive or negative
        spike_amplitude = np.random.uniform(-150, 150)
        spike_width = np.random.randint(1, 4)
        
        # Add spike with slight decay
        for i in range(spike_width):
            if spike_loc + i < num_samples:
                decay_factor = 1.0 / (i + 1)
                emi_noise[spike_loc + i] += spike_amplitude * decay_factor
    
    # Combine all components
    temperature_data[sensor_id] = base_temp + trend + thermal_noise + emi_noise

# Clip to realistic sensor range (-40 to 120°C)
# EMI spikes may push readings out of range briefly
temperature_data = np.clip(temperature_data, -40, 120)

# Round to 1 decimal place (realistic sensor precision)
temperature_data = np.round(temperature_data, 1)

# Save to TXT file (space-delimited, no headers)
# Format: 32 rows × 500 columns
with open('mux_3_data.txt', 'w') as f:
    for sensor_id in range(num_sensors):
        # Write all 500 samples for this sensor on one line
        row_data = ' '.join([f'{temp:.1f}' for temp in temperature_data[sensor_id]])
        f.write(row_data + '\n')
