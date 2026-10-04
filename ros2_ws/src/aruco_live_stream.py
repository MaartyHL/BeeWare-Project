#!/usr/bin/env python3
"""
BeeWare Project - Nœud de vision et de retour vidéo ArUco en direct
Capture la caméra Global Shutter, détecte le marqueur ArUco,
calcule l'erreur de centrage (X, Y) et diffuse la vidéo en direct sur le port 5000.
"""

import time
import io
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import cv2
import numpy as np

try:
    from picamera2 import Picamera2
except ImportError:
    print("Erreur: picamera2 n'est pas installé. Lancez: sudo apt install -y python3-picamera2")
    exit(1)

# Variables partagées pour le streaming vidéo
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
                <title>🐝 BeeWare Drone - Flux Caméra ArUco</title>
                <style>
                    body { font-family: Arial, sans-serif; background: #121212; color: #fff; text-align: center; margin: 0; padding: 20px; }
                    h1 { color: #f39c12; margin-bottom: 5px; }
                    p { color: #aaa; margin-top: 0; }
                    .video-box { display: inline-block; border: 3px solid #f39c12; border-radius: 8px; overflow: hidden; box-shadow: 0 4px 20px rgba(0,0,0,0.6); }
                    img { display: block; max-width: 100%; height: auto; }
                    .stats { margin-top: 15px; font-size: 1.1em; color: #2ecc71; }
                </style>
            </head>
            <body>
                <h1>🐝 BeeWare Project - FPV Global Shutter</h1>
                <p>Détection ArUco en direct (Caméra Sony IMX296)</p>
                <div class="video-box">
                    <img src="/stream.mjpg" width="640" height="480" />
                </div>
                <div class="stats">Flux actif - Présentez un marqueur ArUco (ID 0)</div>
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
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(f'Content-Length: {len(frame_bytes)}\r\n\r\n'.encode('utf-8'))
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b'\r\n')
                    self.wfile.flush()
                    time.sleep(0.03)  # ~30 FPS
            except Exception:
                pass
        elif self.path == '/snapshot.jpg':
            with lock:
                if output_frame is not None:
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(output_frame)))
                    self.end_headers()
                    self.wfile.write(output_frame)
                    return
            self.send_error(503)
            self.end_headers()
        else:
            self.send_error(404)
            self.end_headers()

def start_web_server(port=5000):
    server = HTTPServer(('0.0.0.0', port), StreamingHandler)
    print(f"🌐 Serveur de flux vidéo actif sur : http://172.20.10.6:{port}")
    server.serve_forever()

def main():
    global output_frame, lock

    print("🚀 Initialisation de la caméra Global Shutter (IMX296)...")
    picam2 = Picamera2()
    # Configuration en 640x480 pour une vitesse maximale et très faible latence
    config = picam2.create_video_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)
    print("✅ Caméra prête !")

    # Configuration du détecteur ArUco OpenCV
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        use_new_api = True
    else:
        use_new_api = False

    # Lancement du serveur Web dans un thread séparé
    web_thread = threading.Thread(target=start_web_server, args=(5000,), daemon=True)
    web_thread.start()

    frame_center_x = 640 // 2
    frame_center_y = 480 // 2

    last_log_time = 0

    print("🔍 Recherche de marqueurs ArUco en cours... (Ctrl+C pour quitter)")

    try:
        while True:
            # Capture de l'image
            frame = picam2.capture_array()

            # La caméra est monochrome, convertissons si besoin pour OpenCV
            if len(frame.shape) == 2:
                gray = frame
                display = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                display = frame.copy()

            # Détection ArUco
            if use_new_api:
                corners, ids, rejected = detector.detectMarkers(gray)
            else:
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

            # Dessin de la cible au centre de l'image (croix de visée du drone)
            cv2.drawMarker(display, (frame_center_x, frame_center_y), (100, 100, 100), cv2.MARKER_CROSS, 20, 1)

            now = time.time()

            if ids is not None and len(ids) > 0:
                # Dessin des contours verts du marqueur
                cv2.aruco.drawDetectedMarkers(display, corners, ids)

                # Calcul du centre du premier marqueur détecté
                c = corners[0][0]
                marker_center_x = int(np.mean(c[:, 0]))
                marker_center_y = int(np.mean(c[:, 1]))

                # Calcul des erreurs par rapport au centre de visée du drone
                error_x = marker_center_x - frame_center_x
                error_y = marker_center_y - frame_center_y

                # Ligne de guidage entre le centre caméra et le marqueur
                cv2.line(display, (frame_center_x, frame_center_y), (marker_center_x, marker_center_y), (0, 255, 0), 2)
                cv2.circle(display, (marker_center_x, marker_center_y), 5, (0, 0, 255), -1)

                # Texte sur l'image
                cv2.putText(display, f"ID: {ids[0][0]} | ErrX: {error_x:+d} | ErrY: {error_y:+d}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Log console périodique (toutes les 0.5 secondes)
                if now - last_log_time > 0.5:
                    print(f"🎯 ArUco ID {ids[0][0]} DÉTECTÉ ! Décalage X: {error_x:+4d} px | Décalage Y: {error_y:+4d} px")
                    last_log_time = now
            else:
                cv2.putText(display, "RECHERCHE ARUCO...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                if now - last_log_time > 1.5:
                    print("👀 En attente d'un marqueur ArUco dans le champ...")
                    last_log_time = now

            # Compression JPEG pour le streaming web
            ret, jpeg = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ret:
                with lock:
                    output_frame = jpeg.tobytes()

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nArrêt du programme.")
    finally:
        picam2.stop()

if __name__ == "__main__":
    main()
