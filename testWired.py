import threading
import struct
import os
import time
import queue
import cv2
import numpy as np
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import serial
import serial.tools.list_ports

# --- Configuration ---
BAUD_RATE = 2000000
MAX_CAMERAS = 3
VIDEO_RESOLUTION = (640, 480)
FPS = 15 # Used for the recording file, should match ESP32's target
STALL_TIMEOUT_S = 3 # Seconds before a feed is considered stalled

# --- Protocol Constants ---
FRAME_LEN_FORMAT = "<L"
FRAME_LEN_SIZE = struct.calcsize(FRAME_LEN_FORMAT)
JPEG_START_MARKER = b'\xff\xd8'
JPEG_END_MARKER = b'\xff\xd9'

# --- Thread-safe state management ---
class RecordingState:
    def __init__(self):
        self._is_recording = False
        self._lock = threading.Lock()

    def set(self, recording: bool):
        with self._lock:
            self._is_recording = recording

    def is_recording(self) -> bool:
        with self._lock:
            return self._is_recording

# --- Main Application Class ---
class App:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Multi-Camera Viewer (Direct Stream)")
        self.root.geometry("1000x520")

        self.frame_queue = queue.Queue()
        self.recording_state = RecordingState()
        self.photo_references = [None] * MAX_CAMERAS
        self.last_frame_time = [0] * MAX_CAMERAS

        self._build_gui()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Start the backend thread to find and manage camera connections
        self.serial_thread = threading.Thread(target=self.find_and_run_cameras, daemon=True)
        self.serial_thread.start()

        self.update_gui()

    def _build_gui(self):
        """Creates all the GUI elements."""
        self.video_panels = []
        self.camera_labels = []
        
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True)

        for i in range(MAX_CAMERAS):
            panel_frame = tk.Frame(main_frame, bd=2, relief=tk.SUNKEN)
            panel_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
            
            label = tk.Label(panel_frame, text=f"Camera {i+1}\n(Disconnected)", font=("Helvetica", 14), fg="red")
            label.pack(pady=5)
            self.camera_labels.append(label)

            panel = tk.Label(panel_frame)
            panel.pack()
            self.video_panels.append(panel)

        button_frame = tk.Frame(self.root)
        button_frame.pack(fill=tk.X, pady=10)
        self.record_button = tk.Button(button_frame, text="⚫ Record", font=("Helvetica", 16), bg="lightgrey", command=self.toggle_recording)
        self.record_button.pack()

    def on_closing(self):
        """Handles graceful shutdown."""
        if self.recording_state.is_recording():
            if messagebox.askyesno("Recording Active", "You are currently recording. Do you want to stop and exit?"):
                self.recording_state.set(False)
                time.sleep(0.5)
                self.root.destroy()
        else:
            self.root.destroy()

    def toggle_recording(self):
        """Starts or stops recording for all active streams."""
        if self.recording_state.is_recording():
            self.recording_state.set(False)
            self.record_button.config(text="⚫ Record", bg="lightgrey", fg="black")
            print("[INFO] Recording STOPPED.")
        else:
            self.recording_state.set(True)
            self.record_button.config(text="⏹️ Stop", bg="red", fg="white")
            print("[INFO] Recording STARTED.")

    def update_gui(self):
        """Processes the frame queue and updates the GUI panels."""
        try:
            while not self.frame_queue.empty():
                frame_index, frame_data = self.frame_queue.get_nowait()
                
                self.last_frame_time[frame_index] = time.time()
                
                np_arr = np.frombuffer(frame_data, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if img is not None:
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    img_pil = Image.fromarray(img_rgb)
                    img_tk = ImageTk.PhotoImage(image=img_pil)
                    
                    self.video_panels[frame_index].config(image=img_tk)
                    self.photo_references[frame_index] = img_tk

            for i in range(MAX_CAMERAS):
                if self.camera_labels[i].cget("fg") == "green" and time.time() - self.last_frame_time[i] > STALL_TIMEOUT_S:
                    self.camera_labels[i].config(text=f"{self.camera_labels[i].cget('text').splitlines()[0]}\n(Stalled)", fg="red")
                    print(f"[WARN] Feed from Camera {i+1} has stalled.")

        except queue.Empty:
            pass
        finally:
            self.root.after(30, self.update_gui)

    def find_and_run_cameras(self):
        """Scans for serial ports and launches a dedicated thread for each found ESP32."""
        print("[INFO] Searching for ESP32 cameras on serial ports...")
        all_ports = serial.tools.list_ports.comports()
        
        print("--- Detected Serial Ports ---")
        for port in all_ports:
            print(f"- {port.device}: {port.description}")
        print("-----------------------------")

        esp_ports = [p.device for p in all_ports if 'ACM' in p.device or 'USB' in p.device]
        
        if not esp_ports:
            print("[ERROR] No ESP32 cameras found. Please check connections.")
            return

        for i, port_name in enumerate(esp_ports):
            if i >= MAX_CAMERAS:
                print(f"[WARN] Found more than {MAX_CAMERAS} cameras. Ignoring {port_name}.")
                break
            
            thread = threading.Thread(target=self.read_from_port, args=(port_name, i), daemon=True)
            thread.start()

    def read_from_port(self, port_name, frame_index):
        """Worker function that reads directly from the serial port without a handshake."""
        camera_id_str = os.path.basename(port_name)
        self.camera_labels[frame_index].config(text=f"{camera_id_str}\n(Connecting...)", fg="orange")
        video_writer = None
        output_dir = None

        while True:
            try:
                # The 'with' statement ensures the port is automatically closed on error
                with serial.Serial(port_name, BAUD_RATE, timeout=2) as ser:
                    ser.reset_input_buffer() # Clear any old data
                    self.camera_labels[frame_index].config(text=f"{camera_id_str}\n(Streaming)", fg="green")
                    self.last_frame_time[frame_index] = time.time()

                    # Immediately start reading frame data
                    while True:
                        len_bytes = ser.read(FRAME_LEN_SIZE)
                        if len(len_bytes) < FRAME_LEN_SIZE:
                            raise serial.SerialTimeoutException("Read timeout on frame length")

                        frame_len = struct.unpack(FRAME_LEN_FORMAT, len_bytes)[0]
                        # Sanity check for frame length to detect de-sync
                        if not (0 < frame_len < 100000):
                            print(f"[ERROR] Invalid frame length on {port_name}: {frame_len}. Clearing buffer to re-sync.")
                            ser.reset_input_buffer()
                            continue

                        frame_data = ser.read(frame_len)
                        if len(frame_data) < frame_len:
                            print(f"[ERROR] Incomplete frame on {port_name}. Expected {frame_len}, got {len(frame_data)}.")
                            continue
                        
                        # Data Integrity Check: Verify JPEG markers
                        if not (frame_data.startswith(JPEG_START_MARKER) and frame_data.endswith(JPEG_END_MARKER)):
                            print(f"[ERROR] Corrupt JPEG data on {port_name}. Invalid start/end markers.")
                            continue

                        self.frame_queue.put((frame_index, frame_data))
                        
                        # --- Recording Logic ---
                        is_recording = self.recording_state.is_recording()
                        if is_recording and video_writer is None:
                            if output_dir is None: output_dir = "recordings_" + time.strftime("%Y%m%d-%H%M%S"); os.makedirs(output_dir, exist_ok=True)
                            filename = os.path.join(output_dir, f"{camera_id_str.replace('/','_')}.mp4")
                            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                            video_writer = cv2.VideoWriter(filename, fourcc, FPS, VIDEO_RESOLUTION)
                            print(f"[INFO] Started recording for '{camera_id_str}' to {filename}")

                        if is_recording and video_writer is not None:
                            np_arr = np.frombuffer(frame_data, np.uint8); img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                            if img is not None: video_writer.write(img)

                        if not is_recording and video_writer is not None:
                            video_writer.release(); video_writer = None
                            print(f"[INFO] Saved video for '{camera_id_str}'.")
            
            except serial.SerialException as e:
                self.camera_labels[frame_index].config(text=f"{camera_id_str}\n(Disconnected)", fg="red")
                if video_writer: video_writer.release(); video_writer = None
                print(f"[ERROR] Serial error on {port_name}: {e}. Retrying in 5 seconds...")
                time.sleep(5)

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()