# 🚗 SmartSurround

> **실시간 운전 위험 감지 시스템**  
> **과속 · 난폭 · 졸음운전 감지 후 경고를 제공하는 임베디드 기반 차량 내 스마트 감시 솔루션**

---

## 🧠 프로젝트 개요

**SmartSurround**는 운전 중 발생할 수 있는 위험 상황(과속, 졸음 운전)을 **실시간으로 감지**하고, 차량 내 사용자에게 **경고를 전달**하는 시스템입니다.  
**STM32**, **CAN 통신**, **PC**를 연동하여 차량 전장 아키텍처를 시뮬레이션하였습니다.

---

## 🔧 주요 기능

- **운전 행태 감지**  
  딥러닝 기반 객체 추적 및 분석 알고리즘 구현

- **CAN 통신을 통한 명령 전송**  
  분석 결과를 바탕으로 차량 내 노드(CAN 노드)에 알림 전송

- **하드웨어 경고 시스템**  
  LED, 부저 등을 통해 시각적/청각적 경고 제공

---

## ⚙️ 사용 기술

| 구분           | 기술/장비 |
|----------------|-----------|
| MCU            | STM32F411RE |
| 통신           | CAN (Controller Area Network) |
| 개발 툴        | STM32CubeIDE, YOLOv8, OpenCV |
| 센서           | 부저, LED |

---

## 🧑‍💻 담당 역할

- **하드웨어 회로 설계**
- **CAN 통신 구현**
- **시스템 통합**: MCU ↔ 센서 ↔ PC 전체 흐름 설계 및 디버깅

---

## 📷 시연 이미지

![image](https://github.com/user-attachments/assets/3b589fc1-4870-452a-920a-fdf7b71b944a)<br>
![image](https://github.com/user-attachments/assets/3a059f98-ad40-4cd6-9af1-45d703363d3d)<br>
![image](https://github.com/user-attachments/assets/cc6c4bc8-89c3-4711-871e-a82e4485f5fb)


---
