import socket
import threading
import struct
import os
import time
import queue
import cv2
import numpy as np
import tkinter as tk
from PIL import Image, ImageTk

# --- Configuration ---
HOST = '0.0.0.0'
PORT = 8000
VIDEO_RESOLUTION = (640, 480)
FPS = 10 

# --- Header format ---
HEADER_FORMAT = "<16sLLL"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

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
        self.root.title("ESP32 Multi-Camera Viewer")
        self.root.geometry("1000x520")

        # --- State and Data ---
        self.frame_queue = queue.Queue()
        self.recording_state = RecordingState()
        # **NEW**: A thread-safe dictionary to manage camera slots
        self.camera_slots = {}
        self.slots_lock = threading.Lock()
        
        # --- GUI Elements ---
        self.video_panels = {}
        self.camera_labels = {}
        self.photo_references = {} # To prevent garbage collection of images
        
        main_frame = tk.Frame(root)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Define the initial camera names/slots
        self.slot_ids = ["FORWARD", "LEFT_EYE", "RIGHT_EYE"]
        for i, cam_id in enumerate(self.slot_ids):
            panel_frame = tk.Frame(main_frame, bd=2, relief=tk.SUNKEN)
            panel_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
            
            label = tk.Label(panel_frame, text=f"{cam_id}\n(Disconnected)", font=("Helvetica", 14), fg="red")
            label.pack(pady=5)
            self.camera_labels[cam_id] = label

            panel = tk.Label(panel_frame)
            panel.pack()
            self.video_panels[cam_id] = panel

        # --- Control buttons ---
        button_frame = tk.Frame(root)
        button_frame.pack(fill=tk.X, pady=10)
        self.record_button = tk.Button(button_frame, text="⚫ Record", font=("Helvetica", 16), bg="lightgrey", command=self.toggle_recording)
        self.record_button.pack()

        # --- Start Networking ---
        self.server_thread = threading.Thread(target=self.start_server, daemon=True)
        self.server_thread.start()

        # --- Start GUI Update Loop ---
        self.update_gui()
        
        # --- Handle window closing ---
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def on_closing(self):
        """Handle cleanup when the window is closed."""
        print("Application closing...")
        self.root.destroy()

    def toggle_recording(self):
        if self.recording_state.is_recording():
            self.recording_state.set(False)
            self.record_button.config(text="⚫ Record", bg="lightgrey", fg="black")
            print("🔴 Recording STOPPED.")
        else:
            self.recording_state.set(True)
            self.record_button.config(text="⏹️ Stop", bg="red", fg="white")
            print("🟢 Recording STARTED.")

    def update_gui(self):
        try:
            while not self.frame_queue.empty():
                camera_id, frame_data = self.frame_queue.get_nowait()
                
                if camera_id not in self.video_panels:
                    continue

                np_arr = np.frombuffer(frame_data, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if img is not None:
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    img_pil = Image.fromarray(img_rgb)
                    img_tk = ImageTk.PhotoImage(image=img_pil)
                    
                    self.video_panels[camera_id].config(image=img_tk)
                    self.photo_references[camera_id] = img_tk
                    self.camera_labels[camera_id].config(text=camera_id, fg="green")

        except queue.Empty:
            pass
        finally:
            self.root.after(30, self.update_gui)

    def start_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen(5)
            print(f"🚀 Server listening on port {PORT}. Waiting for cameras...")

            while True:
                conn, addr = s.accept()
                handler_thread = threading.Thread(
                    target=self.handle_client,
                    args=(conn, addr),
                    daemon=True
                )
                handler_thread.start()

    def handle_client(self, conn, addr):
        print(f"  ➡️  New connection from {addr}, waiting for ID...")
        camera_id = None
        video_writer = None
        output_dir = None
        
        try:
            # **CRITICAL CHANGE**: First, read the header to identify the camera
            header_data = conn.recv(HEADER_SIZE)
            if not header_data or len(header_data) < HEADER_SIZE:
                raise ConnectionError("Failed to receive header")
                
            header = struct.unpack(HEADER_FORMAT, header_data)
            camera_id = header[0].decode('utf-8').rstrip('\x00')

            # **CRITICAL CHANGE**: Check if the ID is valid and the slot is free
            with self.slots_lock:
                if camera_id not in self.slot_ids:
                    raise ConnectionError(f"Unknown camera ID: {camera_id}")
                if camera_id in self.camera_slots and self.camera_slots[camera_id] is not None:
                    raise ConnectionError(f"Camera slot for {camera_id} is already taken")
                # Reserve the slot
                self.camera_slots[camera_id] = conn
            
            print(f"✅ Camera '{camera_id}' connected from {addr}")
            
            # Now, process the first frame's data
            frame_len = header[3]
            frame_data = b''
            while len(frame_data) < frame_len:
                chunk = conn.recv(frame_len - len(frame_data))
                if not chunk: break
                frame_data += chunk
            
            if len(frame_data) == frame_len:
                self.frame_queue.put((camera_id, frame_data))

            # --- Main loop to receive subsequent frames ---
            while True:
                header_data = conn.recv(HEADER_SIZE)
                if not header_data or len(header_data) < HEADER_SIZE:
                    break

                header = struct.unpack(HEADER_FORMAT, header_data)
                frame_len = header[3]

                frame_data = b''
                while len(frame_data) < frame_len:
                    chunk = conn.recv(frame_len - len(frame_data))
                    if not chunk: break
                    frame_data += chunk
                
                if len(frame_data) != frame_len: continue
                self.frame_queue.put((camera_id, frame_data))

                # --- Recording Logic ---
                is_currently_recording = self.recording_state.is_recording()

                if is_currently_recording and video_writer is None:
                    if output_dir is None:
                        output_dir = "recordings_" + time.strftime("%Y%m%d-%H%M%S")
                        os.makedirs(output_dir, exist_ok=True)
                    
                    filename = os.path.join(output_dir, f"{camera_id}.mp4")
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    video_writer = cv2.VideoWriter(filename, fourcc, FPS, VIDEO_RESOLUTION)
                    print(f"Writing video for '{camera_id}' to {filename}")

                if is_currently_recording and video_writer is not None:
                    np_arr = np.frombuffer(frame_data, np.uint8)
                    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if img is not None:
                        video_writer.write(img)

                if not is_currently_recording and video_writer is not None:
                    video_writer.release()
                    video_writer = None
                    print(f"Video file for '{camera_id}' saved.")

        except (ConnectionResetError, BrokenPipeError, ConnectionError) as e:
            print(f"💥 Connection issue with {addr}: {e}")
        finally:
            if camera_id:
                # **CRITICAL CHANGE**: Free up the slot when disconnecting
                with self.slots_lock:
                    self.camera_slots[camera_id] = None
                self.camera_labels[camera_id].config(text=f"{camera_id}\n(Disconnected)", fg="red")
                print(f"🔌 Camera '{camera_id}' disconnected.")
            
            if video_writer is not None:
                video_writer.release()
            conn.close()

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()