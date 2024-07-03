from pathlib import Path
import cv2
import matplotlib.pyplot as plt

# Load the video
# Be sure to mount in colab first
video_path = Path("/mnt/data/pso_animation.avi")
cap = cv2.VideoCapture(str(video_path))

# Check if video opened successfully
if not cap.isOpened():
    print("Error opening video file.")

# Parameters for frame extraction
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
frames_to_extract = 10  # Number of frames to extract evenly across the video

# Calculate interval for frame extraction
interval = total_frames // frames_to_extract

# Initialise a list to hold the extracted frames
extracted_frames = []

for i in range(0, total_frames, interval):
    cap.set(cv2.CAP_PROP_POS_FRAMES, i)
    ret, frame = cap.read()
    if ret:
        # Convert the frame from BGR to RGB color space
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        extracted_frames.append(frame_rgb)
    else:
        print(f"Error extracting frame at position {i}.")

# Release the video capture object
cap.release()

# Plot the extracted frames
fig, axs = plt.subplots(2, 5, figsize=(20, 8))

for ax, frame in zip(axs.flat, extracted_frames):
    ax.imshow(frame)
    ax.axis('off')

plt.tight_layout()
plt.show()
