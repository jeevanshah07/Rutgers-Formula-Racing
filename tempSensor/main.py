import numpy as np

for j in range(12):
    np.random.seed(69 + j)

    num_sensors = 32
    num_samples = 500

    temperature_data = np.zeros((num_sensors, num_samples))

    time = np.linspace(0, num_samples - 1, num_samples)

    base_temps = 28 + np.random.uniform(-2, 2, num_sensors)

    for sensor_id in range(num_sensors):
        base_temp = base_temps[sensor_id]

        heating_rate = 0.02 + (0.005 if 8 <= sensor_id <= 23 else 0)
        trend = heating_rate * time

        thermal_noise = np.random.normal(0, 0.5, num_samples)

        emi_noise = np.zeros(num_samples)

        num_spikes = np.random.randint(5, 16)
        spike_locations = np.random.choice(num_samples, size=num_spikes, replace=False)

        for spike_loc in spike_locations:
            spike_amplitude = np.random.uniform(-150, 150)
            spike_width = np.random.randint(1, 4)

            for i in range(spike_width):
                if spike_loc + i < num_samples:
                    decay_factor = 1.0 / (i + 1)
                    emi_noise[spike_loc + i] += spike_amplitude * decay_factor

        temperature_data[sensor_id] = base_temp + trend + thermal_noise + emi_noise

    temperature_data = np.clip(temperature_data, -40, 120)

    temperature_data = np.round(temperature_data, 1)

    with open(f"mux_{j}_data.txt", "w") as f:
        for sensor_id in range(num_sensors):
            f.write("32 500 \n")
            row_data = " ".join([f"{temp:.1f}" for temp in temperature_data[sensor_id]])
            f.write(row_data + "\n")
