#!/usr/bin/env python3
"""
BeeWare Project - Nœud de Vision ArUco & Stream Vidéo FPV
Capture la caméra IMX296 à 60 FPS, diffuse le flux web sur le port 5000,
et transmet en direct les erreurs (X, Y, Taille) au nœud de vol ROS 2 via UDP local.
"""

import time
import socket
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import cv2
import numpy as np

try:
    from picamera2 import Picamera2
except ImportError:
    print("Erreur: picamera2 n'est pas disponible. Utilisez le Python systeme hors de conda.")
    exit(1)

# Socket UDP pour transmettre les coordonnees au noeud ROS 2 (sur localhost:9876)
UDP_IP = "127.0.0.1"
UDP_PORT = 9876
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

output_frame = None
lock = threading.Lock()

class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global output_frame, lock
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            html = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>🐝 BeeWare Drone - FPV ArUco</title>
                <style>
                    body { font-family: Arial, sans-serif; background: #121212; color: #fff; text-align: center; margin: 0; padding: 20px; }
                    h1 { color: #f39c12; margin-bottom: 5px; }
                    .video-box { display: inline-block; border: 3px solid #f39c12; border-radius: 8px; overflow: hidden; }
                    img { display: block; max-width: 100%; height: auto; }
                </style>
            </head>
            <body>
                <h1>🐝 BeeWare Project - FPV Global Shutter</h1>
                <div class="video-box"><img src="/stream.mjpg" width="640" height="480" /></div>
            </body>
            </html>
            """
            self.wfile.write(html.encode('utf-8'))
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with lock:
                        if output_frame is None:
                            time.sleep(0.02)
                            continue
                        frame_bytes = output_frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(frame_bytes)))
                    self.end_headers()
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b'\r\n')
                    self.wfile.flush()
                    time.sleep(0.03)
            except Exception:
                pass
        else:
            self.send_error(404)
            self.end_headers()

def start_web_server(port=5000):
    server = HTTPServer(('0.0.0.0', port), StreamingHandler)
    server.serve_forever()

def main():
    global output_frame, lock

    print("🚀 Initialisation de la caméra Global Shutter (IMX296)...")
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)
    print("✅ Caméra prête !")

    # Lancement du serveur Web en thread séparé
    web_thread = threading.Thread(target=start_web_server, args=(5000,), daemon=True)
    web_thread.start()
    print("🌐 Flux vidéo actif sur : http://172.20.10.6:5000")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        use_new_api = True
    else:
        use_new_api = False

    frame_center_x = 640 // 2
    frame_center_y = 480 // 2

    print("🔍 En attente du marqueur ArUco (ID 0)... (Transmission active vers ROS 2)")

    try:
        while True:
            frame = picam2.capture_array()
            if len(frame.shape) == 2:
                gray = frame
                display = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                display = frame.copy()

            if use_new_api:
                corners, ids, rejected = detector.detectMarkers(gray)
            else:
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

            cv2.drawMarker(display, (frame_center_x, frame_center_y), (100, 100, 100), cv2.MARKER_CROSS, 20, 1)

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(display, corners, ids)
                c = corners[0][0]
                marker_x = int(np.mean(c[:, 0]))
                marker_y = int(np.mean(c[:, 1]))
                side_len = float(np.linalg.norm(c[0] - c[1]))

                err_x = marker_x - frame_center_x
                err_y = marker_y - frame_center_y

                cv2.line(display, (frame_center_x, frame_center_y), (marker_x, marker_y), (0, 255, 0), 2)
                cv2.circle(display, (marker_x, marker_y), 5, (0, 0, 255), -1)

                # Envoi du message UDP vers ROS 2 : "DETECTED,err_x,err_y,side_len"
                msg = f"1,{err_x},{err_y},{side_len:.1f}".encode('utf-8')
                sock.sendto(msg, (UDP_IP, UDP_PORT))
                print(f"[VISION] ArUco vu ! ErrX: {err_x:+4d}px | ErrY: {err_y:+4d}px", end='\r')
            else:
                # Message "aucun marqueur"
                msg = b"0,0,0,0"
                sock.sendto(msg, (UDP_IP, UDP_PORT))

            ret, jpeg = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ret:
                with lock:
                    output_frame = jpeg.tobytes()

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nArrêt.")
    finally:
        picam2.stop()

if __name__ == "__main__":
    main()
