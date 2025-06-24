# 🚗 SmartSurround

> **실시간 운전 위험 감지 시스템**  
> **과속 · 졸음운전 감지 후 경고를 제공하는 임베디드 기반 차량 내 스마트 감시 솔루션**

---

## 🧠 프로젝트 개요

**SmartSurround**는 운전 중 발생할 수 있는 위험 상황(과속, 졸음 운전)을 **실시간으로 감지**하고, 차량 내 사용자에게 **경고를 전달**하는 시스템입니다.  
**STM32**, **CAN 통신**, **PC**를 연동하여 차량 전장 아키텍처를 시뮬레이션하였습니다.

---

## 💡 개발 동기

- 교통사고의 주요 원인 중 하나인 **졸음, 과속운전** 문제를 해결하기 위한 시스템의 필요성
- **CAN 통신**의 실무적 경험

---
## 🔧 주요 기능

- **운전 행태 감지**  
  딥러닝 기반 객체 추적 및 분석 알고리즘 구현

- **CAN 통신을 통한 명령 전송**  
  분석 결과를 바탕으로 차량 내 노드(STM32)에 CAN 메시지 전송

- **하드웨어 경고 시스템**  
   STM32에서 수신된 명령을 바탕으로 LED 점멸 및 부저 음을 통해 시각/청각적 경고 전달

---

## ⚙️ 사용 기술

| 구분           | 기술/장비 |
|----------------|-----------|
| MCU 및 모듈    | STM32F411RE, CANableV2.0, mcp2515(CAN 모듈), 트위스티드 페어케이블 |
| 통신           | CAN (Controller Area Network) |
| 개발 툴        | STM32CubeIDE, YOLOv8, OpenCV |
| 센서           | 부저, LED |

---

## 🧩 시스템 구성도
![image](https://github.com/user-attachments/assets/12acc95a-8785-45d8-b3b5-d860f228798e)

---

## 🔁 시스템 흐름도
![image](https://github.com/user-attachments/assets/be53c489-e83d-401e-95ab-dcb654b36b95)
> 전체 시스템 흐름은 "PC에서 위험 감지 → CAN 메시지 전송 → STM32가 메시지를 수신 및 해석 → LED/부저 동작" 순서로 구성됩니다.  
> 예: CAN ID: 0x123, DATA: 0x01 → LED 점등

---

## 🧑‍💻 담당 역할

- **하드웨어 회로 설계**
- **CAN 통신 구현**
- **시스템 통합**: MCU ↔ 센서 ↔ PC 전체 흐름 설계 및 디버깅

---

## 🚧 개발 중 겪은 문제와 해결

- **문제 상황**
> STM32에서 ID = 0x123, DATA = DE ED BE EF 를 보낼 때는 PC 측에서 제대로 수신. 하지만 PC에서 STM으로 같은 데이터를 보낼때는 ID = 0x91  DATA = EF DE 로 제대로 수신이 되지 않는 문제가 발생
 
- **해결**
>  STM32 와 MCP2515는 SPI 통신을 통해 데이터전송을 하는데 이때 통신속도가 중요 MCP2515는 SPI클럭이 최대 10Mhz까지 인식이 가능하지만 84Mhz로 과도하게 높게 설정되어있었고, Prescaler를 8로 설정해 클럭을 10.5Mhz로 설정 해줌으로써 해결

---
## 📷 시연 이미지

![image](https://github.com/user-attachments/assets/3b589fc1-4870-452a-920a-fdf7b71b944a)<br>
![image](https://github.com/user-attachments/assets/3a059f98-ad40-4cd6-9af1-45d703363d3d)<br>
![image](https://github.com/user-attachments/assets/cc6c4bc8-89c3-4711-871e-a82e4485f5fb)


---
