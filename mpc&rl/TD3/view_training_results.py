import numpy as np
import matplotlib.pyplot as plt

# Step 1: Load the .npy file
file_path = '/home/z/jackal_ws1/src/the-barn-challenge/TD3/results/TD3_CustomEnv2-v0_0.npy'
data = np.load(file_path, allow_pickle=True)

# Step 2: Print the data to inspect
print("Loaded evaluation data:\n")
for i, eval_result in enumerate(data):
    print(f"Evaluation {i+1}: {eval_result}")

# Step 3: Plot the evaluations over time
plt.figure(figsize=(10, 6))
plt.plot(data, marker='o', linestyle='-', color='b')
plt.title('Evaluation Reward over Time')
plt.xlabel('Evaluation Step')
plt.ylabel('Average Reward')
plt.grid(True)

# Save the plot as an image file
output_image_path = 'training_evaluation_plot.png'
plt.savefig(output_image_path)
print(f"Plot saved as {output_image_path}")
