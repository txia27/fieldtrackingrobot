import struct
import threading
import asyncio
import queue
import tkinter as tk
from bleak import BleakClient, BleakScanner

# ==========================================
# --- Configuration ---
# ==========================================
CANVAS_W = 600
CANVAS_H = 400
PIXY_W = 316   # Pixy2 resolution width
PIXY_H = 208   # Pixy2 resolution height

DEVICE_NAME = "JDY-23"  # Change to match your module's advertised name
SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
CHAR_UUID    = "0000ffe1-0000-1000-8000-00805f9b34fb"

# Thread-safe queue: BLE thread pushes raw bytes, tkinter thread pulls them
ble_queue = queue.Queue()

# ==========================================
# --- BLE Thread (async bleak) ---
# ==========================================
async def ble_main():
    print("Scanning for BLE device...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10)

    if device is None:
        print(f"ERROR: Could not find device named '{DEVICE_NAME}'")
        print("Make sure the JDY-23 is powered on and advertising.")
        return

    print(f"Found {device.name} [{device.address}]. Connecting...")

    async with BleakClient(device.address) as client:
        print(f"Connected to {device.name}")

        def notification_handler(sender, data):
            ble_queue.put(bytes(data))

        await client.start_notify(CHAR_UUID, notification_handler)
        print("Subscribed to notifications. Receiving data...")

        # Keep the connection alive until the program exits
        while client.is_connected:
            await asyncio.sleep(0.1)

def run_ble_loop():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(ble_main())

# ==========================================
# --- Tkinter GUI Setup ---
# ==========================================
root = tk.Tk()
root.title("Pixy2 BLE Vision Decoder")
root.geometry(f"{CANVAS_W}x{CANVAS_H + 40}")
canvas = tk.Canvas(root, width=CANVAS_W, height=CANVAS_H, bg="black")
canvas.pack()

detection_rect = canvas.create_rectangle(0, 0, 0, 0, fill="", outline="", width=2)
status_text = canvas.create_text(CANVAS_W / 2, 20, text="Waiting for BLE data...", fill="white", font=("Arial", 14, "bold"))
debug_text = canvas.create_text(CANVAS_W / 2, CANVAS_H - 15, text="", fill="gray", font=("Consolas", 10))

# Rolling buffer for fragmented BLE packets
packet_buffer = bytearray()

# ==========================================
# --- The Decoder Loop ---
# ==========================================
def update_vision():
    global packet_buffer

    # 1. Drain the queue of all available BLE chunks
    while not ble_queue.empty():
        data = ble_queue.get_nowait()
        raw_hex = ' '.join(f'{b:02X}' for b in data)
        print(f"[RAW] {len(data)} bytes: {raw_hex}")
        packet_buffer.extend(data)

    # 2. Hunt for Pixy2 Sync Word in the buffer
    # Block sync: AF C1 21
    parsed = False
    while len(packet_buffer) >= 20:
        sync_found = False

        for i in range(len(packet_buffer) - 2):
            if (packet_buffer[i] == 0xAF and packet_buffer[i+1] == 0xC1 and packet_buffer[i+2] == 0x21) or \
               (packet_buffer[i] == 0xC1 and packet_buffer[i+1] == 0xAF and packet_buffer[i+2] == 0x21):
                sync_found = True
                packet_buffer = packet_buffer[i:]
                break

        if not sync_found:
            packet_buffer = packet_buffer[-3:]
            break

        # 3. Extract the Payload
        if len(packet_buffer) >= 6:
            payload_length = packet_buffer[3]

            if len(packet_buffer) >= 6 + payload_length:
                payload = packet_buffer[6 : 6 + payload_length]

                if payload_length >= 14:
                    sig, x, y, width, height = struct.unpack('<HHHHH', payload[0:10])

                    print(f"[DECODED] Sig:{sig} X:{x} Y:{y} W:{width} H:{height}")

                    # Scale Pixy2 coordinates to canvas
                    scale_x = CANVAS_W / PIXY_W
                    scale_y = CANVAS_H / PIXY_H
                    cx = x * scale_x
                    cy = y * scale_y
                    hw = (width * scale_x) / 2
                    hh = (height * scale_y) / 2

                    # Pick color based on signature
                    if sig == 1:
                        color = "red"
                        label = "Red"
                    elif sig == 2:
                        color = "green"
                        label = "Green"
                    elif sig == 3:
                        color = "blue"
                        label = "Blue"
                    elif sig == 4:
                        color = "yellow"
                        label = "Yellow"
                    else:
                        color = "white"
                        label = f"Sig {sig}"

                    canvas.coords(detection_rect, cx - hw, cy - hh, cx + hw, cy + hh)
                    canvas.itemconfig(detection_rect, fill=color, outline="white")
                    canvas.itemconfig(status_text, text=f"{label} Detected (W:{width} H:{height})", fill=color)
                    canvas.itemconfig(debug_text, text=f"Sig:{sig} X:{x} Y:{y} W:{width} H:{height}")
                    parsed = True

                packet_buffer = packet_buffer[6 + payload_length:]
            else:
                break
        else:
            break

    root.after(20, update_vision)

# ==========================================
# --- Launch ---
# ==========================================
# Start BLE in a daemon background thread
ble_thread = threading.Thread(target=run_ble_loop, daemon=True)
ble_thread.start()

# Start tkinter in the main thread
root.after(100, update_vision)
root.mainloop()
