#!/usr/bin/env python3
"""
Générateur de marqueurs ArUco pour le BeeWare Project.
Génère une image PNG d'un marqueur prêt à être imprimé ou affiché sur un écran de smartphone.
"""

import cv2
import numpy as np
import os

def generate_aruco(marker_id=0, size_pixels=600, output_path="aruco_id0.png"):
    # Dictionnaire ArUco standard 4x4 (très robuste et rapide)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    
    # Création du marqueur avec bordure blanche
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marker_img = cv2.aruco.generateImageMarker(dictionary, marker_id, size_pixels)
    else:
        marker_img = cv2.aruco.drawMarker(dictionary, marker_id, size_pixels)
    
    # Ajout d'une bordure blanche (quiet zone indispensable pour la détection)
    border_size = 50
    bordered_img = cv2.copyMakeBorder(
        marker_img,
        border_size, border_size, border_size, border_size,
        cv2.BORDER_CONSTANT,
        value=255
    )
    
    cv2.imwrite(output_path, bordered_img)
    print(f"[OK] Marqueur ArUco ID {marker_id} genere avec succes : {output_path}")

if __name__ == "__main__":
    generate_aruco(marker_id=0, output_path="aruco_id0.png")
