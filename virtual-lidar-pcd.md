# Virtual LiDAR – PCD export & streaming

This guide expands on the summary in the [README](../README.md) and walks through practical workflows for the exported point clouds.

## Performance warnings

- Each frame export writes into a temporary file before renaming it to the target `latest.pcd`. Even with the built-in limit of one write every 0.25 s (≈4 Hz) this results in noticeable I/O activity, especially on SSDs with limited endurance. 【F:scripts/driver_assistance_angelo234/lidarPcdPublisher.lua†L69-L144】
- When you need live data with the lowest possible latency (for example to feed another visualizer), rely on the TCP stream, which delivers frames without touching the disk and enables `tcp-nodelay` to actively reduce queuing. 【F:scripts/driver_assistance_angelo234/lidarPcdStreamServer.lua†L44-L118】

## PCD header structure

When exporting to disk the module uses the binary PCD format with the following header and payload layout:

1. The `FIELDS x y z intensity`, `SIZE 4 4 4 4`, `TYPE F F F F`, and `COUNT 1 1 1 1` declarations announce four 32-bit floating-point values in the order `x`, `y`, `z`, `intensity`. 【F:scripts/driver_assistance_angelo234/lidarPcdPublisher.lua†L205-L233】
2. Point values are packed with `string.pack('<ffff', ...)`, meaning little-endian `float32` tuples. 【F:scripts/driver_assistance_angelo234/lidarPcdPublisher.lua†L24-L55】
3. The header also contains the sensor orientation and position (`VIEWPOINT`), derived from the vehicle transform and written as a quaternion. 【F:scripts/driver_assistance_angelo234/lidarPcdPublisher.lua†L225-L236】
4. Using the information above you can recreate a complete header outside the game if a client application requires it—see the examples below.

## Intensity channels

The export adds an `intensity` channel that categorizes points for quick filtering: the main scan (1.0), samples classified as ground (0.2), and samples outlining the player vehicle (0.8). You can map these values to custom colors or masks in your tooling. 【F:scripts/driver_assistance_angelo234/lidarPcdPublisher.lua†L200-L239】

## TCP frame format

- The server listens on `127.0.0.1:23511` until you override the host or port in the console. Each connected client receives data in non-blocking mode with `tcp-nodelay` to minimise latency. 【F:scripts/driver_assistance_angelo234/extension.lua†L84-L125】【F:scripts/driver_assistance_angelo234/lidarPcdStreamServer.lua†L44-L79】
- Every frame starts with an ASCII line `PCD <length>\n`, where `<length>` is the byte size of the upcoming binary blob. 【F:scripts/driver_assistance_angelo234/lidarPcdStreamServer.lua†L90-L113】
- The header is followed by raw binary data (`float32` ×4 per point) in the exact same layout as the file export. 【F:scripts/driver_assistance_angelo234/lidarPcdPublisher.lua†L24-L55】【F:scripts/driver_assistance_angelo234/lidarPcdPublisher.lua†L200-L239】
- If no fresh frame is available, the server sends `PING\n` once per second as a heartbeat. Clients should ignore these lines. 【F:scripts/driver_assistance_angelo234/lidarPcdStreamServer.lua†L70-L88】

## Client examples

### Python: subscribe to the TCP stream with Open3D

```python
import socket
import numpy as np
import open3d as o3d

HOST, PORT = "127.0.0.1", 23511
INTENSITY_COLORS = {
    0.2: [0.3, 0.7, 0.3],  # ground
    0.8: [0.9, 0.6, 0.1],  # vehicle
    1.0: [0.2, 0.5, 1.0],  # main scan
}

def recv_exact(sock, size):
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("Stream closed")
        data.extend(chunk)
    return bytes(data)

def read_frame(sock):
    header = bytearray()
    while not header.endswith(b"\n"):
        chunk = sock.recv(1)
        if not chunk:
            raise ConnectionError("Stream closed")
        header.extend(chunk)
    if header == b"PING\n":
        return None
    _, length = header.split()
    payload = recv_exact(sock, int(length))
    points = np.frombuffer(payload, dtype="<f4").reshape(-1, 4)
    xyz = points[:, :3]
    intensity = points[:, 3]
    colors = np.array([INTENSITY_COLORS.get(float(i), [1.0, 1.0, 1.0]) for i in intensity], dtype=np.float32)
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)
    cloud.colors = o3d.utility.Vector3dVector(colors)
    return cloud

with socket.create_connection((HOST, PORT)) as s:
    while True:
        cloud = read_frame(s)
        if cloud is None:
            continue
        o3d.visualization.draw_geometries([cloud])
```

The script converts each frame into an `open3d.geometry.PointCloud`, colors points by intensity, and renders them in a simple window.

### Python: capture a frame for the PCL library

```python
import socket
import pathlib

HOST, PORT = "127.0.0.1", 23511
OUTPUT = pathlib.Path("frame.pcd")

def recv_line(sock):
    line = bytearray()
    while not line.endswith(b"\n"):
        chunk = sock.recv(1)
        if not chunk:
            raise ConnectionError
        line.extend(chunk)
    return bytes(line)

def recv_exact(sock, size):
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError
        data.extend(chunk)
    return bytes(data)

with socket.create_connection((HOST, PORT)) as s:
    while True:
        line = recv_line(s)
        if line == b"PING\n":
            continue
        size = int(line.split()[1])
        payload = recv_exact(s, size)
        points = len(payload) // (4 * 4)
        header = f"""# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {points}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {points}\nDATA binary\n"""
        OUTPUT.write_bytes(header.encode("ascii") + payload)
        print(f"Saved {points} points to {OUTPUT}")
        break
```

Adjust the header if your downstream application needs the real sensor pose—the structure is documented above.

### Working with the export file (tail)

For lightweight processing without custom code you can monitor `latest.pcd` with a command such as `tail -F "Documents/BeamNG.drive/virtual_lidar/latest.pcd"`. Every overwrite indicates a new frame; automation scripts can react to the change by loading the file (for example through `open3d.io.read_point_cloud`). Because of the I/O load described earlier, prefer the TCP stream whenever you need near-real-time data. 【F:scripts/driver_assistance_angelo234/lidarPcdPublisher.lua†L69-L144】【F:scripts/driver_assistance_angelo234/lidarPcdStreamServer.lua†L64-L118】
