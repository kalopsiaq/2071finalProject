import numpy as np
import wave
import serial

# --- Config ---
PORT = 'COM5'          # Change to your STM's port (e.g. '/dev/ttyACM0' on Linux/Mac)
BAUD_RATE = 115200
SAMPLE_RATE = 10000
FILENAME = 'output.wav'

# --- Connect to STM32 over UART ---
ser = serial.Serial(PORT, BAUD_RATE, timeout=2)
print("Connected. Recording...")

# --- Collect audio samples for 5 seconds ---
data = []
for i in range(5 * SAMPLE_RATE):
    byte = ser.read(1)   # Read 1 byte from UART
    data.append(byte[0]) # Get the integer value and append

ser.close()
print("Done recording. Processing...")

# --- Convert to numpy array and normalise to full uint8 range ---
data = np.array(data)
data = (data - data.min()) / data.max()  # Scale to 0-1
data = data * 255                         # Scale to 0-255
data = data.astype(np.uint8)              # Convert to uint8

# --- Write to WAV file ---
with wave.open(FILENAME, 'wb') as wf:
    wf.setnchannels(1)          # Mono audio
    wf.setsampwidth(1)          # 8 bits (1 byte) per sample
    wf.setframerate(SAMPLE_RATE)
    wf.writeframes(data.tobytes())

print(f"Saved to {FILENAME}")