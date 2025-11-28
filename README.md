# ProcesamientoDigitaldeSe-ales
Visualizador 3D en tiempo real para MPU6050 con ESP32. Incluye filtrado Kalman, fusión sensorial, auto-calibración y un modelo 3D interactivo. Muestra pitch, roll y yaw con gráficas dinámicas y una interfaz PyQt5 optimizada para análisis de movimiento.
# 🛰️ IMU MPU6050 3D Simulator – ESP32 + Python  
Visualización 3D en tiempo real para MPU6050 usando ESP32 y un entorno avanzado en Python (PyQt5 + PyOpenGL + PyQtGraph). Incluye filtrado Kalman, fusión sensorial, auto-calibración y gráficas dinámicas.

## ✨ Características principales
- 🔧 **Procesamiento avanzado IMU**
  - Filtro Kalman por eje  
  - Fusión sensorial acelerómetro + giroscopio  
  - Corrección automática de bias  
  - Cálculo optimizado de Pitch, Roll y Yaw

- 🧭 **Auto-Calibración Inteligente**
  - Detección de inmovilidad  
  - Ajuste automático de offsets  
  - Prevención de calibración errónea por vibración

- 🖥️ **Interfaz Gráfica Profesional (PyQt5)**
  - Tema oscuro  
  - Gráficas de acelerómetro, giroscopio y ángulos  
  - Instrumentos tipo gauge  
  - Selección automática de puerto serial

- ✈️ **Simulación 3D (PyOpenGL + PyQtGraph)**
  - Modelo 3D completo (avión)  
  - Rotaciones reales: Roll → Pitch → Yaw  
  - Interpolación suave  
  - Ejes, flechas y rejilla espacial

- 🔌 **Compatibilidad con ESP32**
  - Comunicación serial estable  
  - Soporte para cualquier firmware que envíe datos del MPU6050 en ASCII  
  - Manejo de paquetes incompletos

## 🧩 Tecnologías utilizadas
- Python 3  
- PyQt5  
- PyQtGraph  
- PyOpenGL  
- NumPy  
- PySerial
- ESP32-WROOM-32  
- MPU6050 (I2C)

