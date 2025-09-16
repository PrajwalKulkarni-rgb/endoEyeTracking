import socket
import threading
import struct
import os
import time

# --- Configuration ---
HOST = '0.0.0.0'  # Listen on all network interfaces
PORT = 8000
OUTPUT_DIR = "recordings_" + time.strftime("%Y%m%d-%H%M%S")

# --- Header format must EXACTLY match the C++ struct ---
# < = Little-endian, 16s = 16-char string, L = unsigned long (4 bytes)
HEADER_FORMAT = "<16sLLL" # Added one more L for frame_len
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

def handle_client(conn, addr):
    """Handles one camera connection."""
    print(f"✅ Camera connected from {addr}")
    
    try:
        while True:
            # 1. Receive the fixed-size header first
            header_data = conn.recv(HEADER_SIZE)
            if not header_data:
                break

            header = struct.unpack(HEADER_FORMAT, header_data)
            camera_id = header[0].decode('utf-8').rstrip('\x00')
            timestamp_sec = header[1]
            timestamp_usec = header[2]
            frame_len = header[3]

            # 2. Receive the image data, which has a variable length
            frame_data = b''
            while len(frame_data) < frame_len:
                chunk = conn.recv(frame_len - len(frame_data))
                if not chunk:
                    break
                frame_data += chunk
            
            if len(frame_data) != frame_len:
                print(f"⚠️ Incomplete frame from {camera_id}. Skipping.")
                continue

            # 3. Save the frame to an organized folder structure
            camera_dir = os.path.join(OUTPUT_DIR, camera_id)
            os.makedirs(camera_dir, exist_ok=True)
            
            # The filename IS the timestamp, ensuring perfect chronological order
            filename = f"{timestamp_sec}_{timestamp_usec:06d}.jpg"
            filepath = os.path.join(camera_dir, filename)

            with open(filepath, 'wb') as f:
                f.write(frame_data)
            
            print(f"Saved: {filepath}")

    except ConnectionResetError:
        print(f"💥 Connection lost with {addr}")
    finally:
        print(f"🔌 Disconnecting {addr}")
        conn.close()

def start_server():
    """Starts the main server to listen for cameras."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"🎥 Output directory: {OUTPUT_DIR}")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(3) # Listen for up to 3 connections
        print(f"🚀 Server listening on port {PORT}. Waiting for cameras...")
        
        threads = []
        try:
            while True:
                conn, addr = s.accept()
                thread = threading.Thread(target=handle_client, args=(conn, addr))
                thread.start()
                threads.append(thread)
        except KeyboardInterrupt:
            print("\nShutting down server.")
        finally:
            for t in threads:
                t.join(timeout=1)


if __name__ == "__main__":
    start_server()