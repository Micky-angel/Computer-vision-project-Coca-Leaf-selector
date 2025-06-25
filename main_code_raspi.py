import cv2 as opencv
from abc import ABC, abstractmethod
from color_detection import color_detection_mask
from ultralytics import YOLO
from gpiozero import Button, LED
from gpiozero.pins.lgpio import LGPIOFactory
import time
import threading
import numpy as np
import asyncio
import websockets
import json

# === Eventos de sincronización entre hilos ===
flag_procesar = threading.Event()
flag_enviar = threading.Event()
tosend = ""

# === Clases base de captura de video ===
class video_capture_abs(ABC):
    @abstractmethod
    def start_camera(self): pass

    @abstractmethod
    def stop_camera(self): pass

    @abstractmethod
    def camera_visualization(self): pass

class video_capture(video_capture_abs):
    def __init__(self, camera):
        self.camera = camera
        self.enabled = False

    def start_camera(self):
        self.enabled = True
        self.camera_visualization()

    def stop_camera(self):
        self.enabled = False

    def camera_visualization(self):
        while self.enabled:
            _, frame = self.camera.read()
            frame = opencv.resize(frame, (320, 240))
            opencv.imshow("camera", frame)
            if opencv.waitKey(1) & 0xFF == ord('q'):
                self.stop_camera()

# === Captura + YOLO + WebSocket ===
class video_capture_with_filters(video_capture):
    factory = LGPIOFactory()
    boton = Button(17, pull_up=False, pin_factory=factory)

    def __init__(self, camera, filtro):
        super().__init__(camera)

    def pixel_a_mm_superior_derecha(self, cx_px, cy_px):
        factor_x = 380 / 640
        factor_y = 220 / 400
        cx_mm = (640.00 - cx_px) * factor_x
        cy_mm = cy_px * factor_y
        return cx_mm, cy_mm

    def dibujar_grilla_con_mm(self, frame, cols=8, rows=6):
        height, width = frame.shape[:2]
        mm_width = 380
        mm_height = 220

        px_por_col = width // cols
        px_por_row = height // rows
        mm_por_col = mm_width // cols
        mm_por_row = mm_height // rows

        for i in range(1, cols):
            x = i * px_por_col
            opencv.line(frame, (x, 0), (x, height), (100, 100, 100), 1)
            # Invertir etiqueta de X
            valor_mm_x = mm_width - (i * mm_por_col)
            opencv.putText(frame, f"{valor_mm_x}", (x + 2, 12), opencv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        for j in range(1, rows):
            y = j * px_por_row
            opencv.line(frame, (0, y), (width, y), (100, 100, 100), 1)
            opencv.putText(frame, f"{j * mm_por_row}", (2, y - 2), opencv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
        return frame

    def camera_visualization(self):
        global flag_procesar, flag_enviar, tosend
        while self.enabled:
            _, frame = self.camera.read()
            self.dibujar_grilla_con_mm(frame)
            if opencv.waitKey(1) & 0xFF == ord('q'):
                self.stop_camera()
            opencv.imshow("Camera", frame)

            if flag_procesar.is_set():
                frame = opencv.medianBlur(frame, 5)
                modelo = YOLO("bestcocaY11l_35.pt")
                resultados = modelo(frame)

                detecciones = []
                colores = {
                    0: (0, 0, 255),
                    1: (0, 255, 0),
                    2: (255, 0, 0)
                }

                for box in resultados[0].boxes:
                    coords = box.xyxy[0].tolist()
                    x1, y1, x2, y2 = map(int, coords)
                    conf = box.conf[0].item()
                    cls = int(box.cls[0].item())
                    pixel_count = (x2 - x1) * (y2 - y1)

                    if pixel_count < 1100 and cls != 0:
                        cls = 2

                    cx = (coords[0] + coords[2]) / 2
                    cy = (coords[1] + coords[3]) / 2
                    cx_mm, cy_mm = self.pixel_a_mm_superior_derecha(cx, cy)

                    # Ajuste según origen local
                    cx_mm = int((cx_mm - 67) * 100)
                    cy_mm = int((cy_mm + 75) * 100)
                    #print(cx,cy,conf)
                    if conf < 0.70:
                        continue
                    detecciones.append({
                        "x": cx_mm,
                        "y": cy_mm,
                        "clase": cls
                    })

                    opencv.rectangle(frame, (x1, y1), (x2, y2), colores.get(cls, (255, 255, 255)), 2)
                    opencv.putText(frame, f"Clase {cls}, {pixel_count} {round(conf,2)}", (x1, y1 - 10),
                                   opencv.FONT_HERSHEY_SIMPLEX, 0.3, colores[cls], 1)

                self.dibujar_grilla_con_mm(frame)
                tosend = json.dumps({"detecciones": detecciones})
                flag_procesar.clear()
                flag_enviar.set()
                opencv.imshow("Detecciones", frame)
                opencv.waitKey(0)
                opencv.destroyAllWindows()

# === Hilo de cámara ===
def iniciar_camara():
    cam = opencv.VideoCapture(0)
    cam_obj = video_capture_with_filters(cam, "None")
    cam_obj.start_camera()
    cam_obj.camera_visualization()

# === Hilo WebSocket (cliente) ===
async def enviar_websocket():
    global flag_enviar, tosend, flag_procesar
    uri = "ws://192.168.172.206:81"

    try:
        async with websockets.connect(uri, ping_interval=None) as websocket:
            print("[WebSocket] Conectado al servidor ESP32")
            while True:
                try:
                    respuesta = await asyncio.wait_for(websocket.recv(), timeout=0.1)
                    if respuesta == "1":
                        print("[WebSocket] Señal de procesamiento recibida")
                        flag_procesar.set()
                except asyncio.TimeoutError:
                    pass
                
                if flag_enviar.is_set():
                    try:
                        await websocket.send(tosend)
                        print(f"[WebSocket] Enviado: {tosend}")
                        respuesta = await asyncio.wait_for(websocket.recv(), timeout=0.1)
                        print(f"[WebSocket] Respuesta ESP32: {respuesta}")
                    except Exception as e:
                        print(f"[WebSocket ERROR] {e}")
                    flag_enviar.clear()

                await asyncio.sleep(0.05)
    except Exception as e:
        print(f"[WebSocket ERROR] No se pudo conectar: {e}")

def iniciar_websocket_loop():
    asyncio.run(enviar_websocket())

# === MAIN ===
if __name__ == "__main__":
    hilo1 = threading.Thread(target=iniciar_camara)
    hilo2 = threading.Thread(target=iniciar_websocket_loop)

    hilo1.start()
    hilo2.start()
