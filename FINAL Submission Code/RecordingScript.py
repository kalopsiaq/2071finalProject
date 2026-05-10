import csv
import time
import wave
import struct
from pathlib import Path

import serial
import matplotlib.pyplot as plt

# Two-byte packet marker used to find the start of each STM packet.
MAGIC = b"\xA5\x5A"

# Packet layout:
# byte 0-1: magic
# byte 2: flags
# byte 3: distance trigger flag
# byte 4 onward: packed ADC payload
HEADER_BYTES = 4
PAYLOAD_BYTES = 384
PACKET_BYTES = HEADER_BYTES + PAYLOAD_BYTES

# Each payload contains 256 ADC samples packed as 12-bit values.
SAMPLES_PER_PACKET = 256
# Index of the ultrasonic/distance flag inside the packet header.
DISTANCE_INDEX = 3


def read_packet(ser):
    """Read one complete packet from the serial stream."""
    while True:
        # Search for the first magic byte.
        b0 = ser.read(1)

        # Timeout or no data available.
        if not b0:
            return None

        # Ignore bytes until the first magic byte is found.
        if b0 != MAGIC[:1]:
            continue

        # Check that the next byte is the second magic byte.
        b1 = ser.read(1)

        if b1 != MAGIC[1:2]:
            continue

        # Read the remaining two header bytes: flags and distance flag.
        header_rest = ser.read(2)

        if len(header_rest) != 2:
            return None

        flags = header_rest[0]
        distance = header_rest[1]

        # Reject false packet headers by requiring both header fields to be 0 or 1.
        if flags not in (0, 1) or distance not in (0, 1):
            continue

        # Read the fixed-size ADC payload
        payload = ser.read(PAYLOAD_BYTES)

        if len(payload) != PAYLOAD_BYTES:
            return None

        # Return the full packet: magic, header, and payload.
        return MAGIC + header_rest + payload


def unpack_adc_payload(payload):
    """Convert packed 12-bit ADC payload bytes into a list of integer samples."""
    samples = []

    # Every 3 bytes contain two 12-bit samples.
    for i in range(0, len(payload), 3):
        b0 = payload[i]
        b1 = payload[i + 1]
        b2 = payload[i + 2]

        # First sample uses b0 plus the lower half of b1.
        sample0 = b0 | ((b1 & 0x0F) << 8)

        # Second sample uses the higher half of b1 plus b2.
        sample1 = ((b1 >> 4) & 0x0F) | (b2 << 4)

        samples.append(sample0)
        samples.append(sample1)

    return samples


def packet_to_samples(packet):
    """Extract and unpack the ADC payload from one packet."""
    return unpack_adc_payload(packet[HEADER_BYTES:])


def adc_to_pcm16(samples):
    """Convert unsigned 12-bit ADC samples into signed 16-bit WAV audio."""
    if not samples:
        return b""

    center = sum(samples) / len(samples)
    # Estimate the DC offset from this recording to center the waveform.
    pcm = bytearray()

    for sample in samples:
        # Convert ADC sample to signed audio and scale 12-bit range to 16-bit range.
        signed = int((sample - center) * 16)

        # Clamp to valid signed 16-bit range.
        signed = max(-32768, min(32767, signed))

        # Store as 16-bit PCM.
        pcm.extend(struct.pack("<h", signed))

    return bytes(pcm)


def save_csv(path, samples, sample_rate):
    """Save raw ADC samples to a CSV file."""
    with open(path, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["recorded_sampling_rate", sample_rate])
        writer.writerow(["index", "time_s", "adc_sample"])

        for i, sample in enumerate(samples):
            writer.writerow([i, i / sample_rate, sample])


def save_wav(path, samples, sample_rate):
    """Save samples as a mono 16-bit WAV file."""
    with wave.open(str(path), "wb") as wav_file:
        wav_file.setnchannels(1)    
        wav_file.setsampwidth(2)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(adc_to_pcm16(samples))


def save_png(path, samples, sample_rate):
    """Save a plot of ADC sample value against time."""
    times = [i / sample_rate for i in range(len(samples))]

    plt.figure(figsize=(12, 4))
    plt.plot(times, samples, linewidth=0.8)
    plt.xlabel("Time (s)")
    plt.ylabel("ADC sample")
    plt.title("Recorded Audio")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def choose_output_formats():
    """Ask the user which output files should be generated."""
    print("\nOutput formats:")
    print("1. CSV only")
    print("2. WAV only")
    print("3. PNG only")
    print("4. CSV + WAV")
    print("5. CSV + PNG")
    print("6. WAV + PNG")
    print("7. CSV + WAV + PNG")

    while True:
        choice = input("Choose output format option: ").strip()

        if choice == "1":
            return ["csv"]
        if choice == "2":
            return ["wav"]
        if choice == "3":
            return ["png"]
        if choice == "4":
            return ["csv", "wav"]
        if choice == "5":
            return ["csv", "png"]
        if choice == "6":
            return ["wav", "png"]
        if choice == "7":
            return ["csv", "wav", "png"]

        print("Invalid option. Choose 1 to 7.")


def save_recording(base_name, samples, sample_rate, output_formats):
    """Save one recording using the selected output formats."""
    base = Path(base_name)

    if "csv" in output_formats:
        save_csv(base.with_suffix(".csv"), samples, sample_rate)
        print(f"Saved {base.with_suffix('.csv')}")

    if "wav" in output_formats:
        save_wav(base.with_suffix(".wav"), samples, sample_rate)
        print(f"Saved {base.with_suffix('.wav')}")

    if "png" in output_formats:
        save_png(base.with_suffix(".png"), samples, sample_rate)
        print(f"Saved {base.with_suffix('.png')}")

    # Print a small summary to help check signal level and clipping.
    if samples:
        print(f"Samples: {len(samples)}")
        print(f"Duration: {len(samples) / sample_rate:.2f} s")
        print(f"Min ADC: {min(samples)}")
        print(f"Max ADC: {max(samples)}")


def record_manual(ser, duration_s, sample_rate, output_formats):
    """Record continuously for a fixed duration."""
    samples = []
    start_time = time.time()

    # Clear old serial data so the recording starts from fresh incoming packets.
    ser.reset_input_buffer()

    print("Recording manual mode...")

    # Record for slightly longer, then crop to the requested sample count.
    # This helps avoid startup/partial-buffer samples appearing at the beginning.
    while time.time() - start_time < duration_s + 1:
        packet = read_packet(ser)

        if packet is None:
            continue

        samples.extend(packet_to_samples(packet))

    # Keep exactly the requested number of samples.
    target_samples = int(duration_s * sample_rate)

    if len(samples) > target_samples:
        samples = samples[-target_samples:]

    save_recording("manual_recording", samples, sample_rate, output_formats)

def record_distance_mode(ser, sample_rate, output_formats):
    """Record automatically while the distance trigger byte is high."""
    record_index = 0
    recording = False
    samples = []

    # Clear old serial data before waiting for the trigger.
    ser.reset_input_buffer()

    print("Distance mode running. Press Ctrl+C to stop.")

    while True:
        packet = read_packet(ser)

        if packet is None:
            continue

        distance_flag = packet[DISTANCE_INDEX]

        # Start a new recording when the distance flag goes high.
        if distance_flag == 1 and not recording:
            recording = True
            samples = []
            print(f"Started distance{record_index}")

        # Store samples only while recording is active.
        if recording:
            samples.extend(packet_to_samples(packet))

        # Stop and save when the distance flag goes low.
        if distance_flag == 0 and recording:
            recording = False

            if samples:
                save_recording(f"distance{record_index}", samples, sample_rate, output_formats)
                record_index += 1

            samples = []


def print_menu():
    print("\n==============================")
    print(" STM Audio Recorder CLI")
    print("==============================")
    print("1. Manual Recording Mode")
    print("2. Distance Trigger Mode")
    print("3. Quit")
    print("==============================")


def prompt_with_default(prompt, default, cast):
    raw = input(f"{prompt} [{default}]: ").strip()

    if raw == "":
        return default

    return cast(raw)


def main():
    """Open the serial port and run the interactive recorder menu."""
    port = prompt_with_default("COM port", "COM4", str)
    baud_rate = prompt_with_default("Baud rate", 921600, int)
    sample_rate = prompt_with_default("Sampling frequency", 44100, int)
    
    # Open STM serial connection.
    with serial.Serial(port, baud_rate, timeout=1) as ser:
        ser.reset_input_buffer()

        while True:
            print_menu()
            mode = input("Select mode, 1/manual, 2/distance, or 3/quit: ").strip().lower()

            if mode == "1" or mode == "manual":
                duration_s = float(input("Duration in seconds: ").strip())
                output_formats = choose_output_formats()
                record_manual(ser, duration_s, sample_rate, output_formats)

            elif mode == "2" or mode == "distance":
                output_formats = choose_output_formats()

                try:
                    record_distance_mode(ser, sample_rate, output_formats)
                except KeyboardInterrupt:
                    print("\nExited distance mode.")

            elif mode == "3" or mode == "quit":
                print("Exiting program.")
                break

            else:
                print("Invalid mode. Choose 1, 2, or 3.")


if __name__ == "__main__":
    main()