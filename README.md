
# 基于 STM32 + MPU9250 + 卡尔曼滤波 + 4G DTU + 腾讯云 的远程姿态监测系统

## 项目简介

本项目设计并实现了一套远程姿态监测系统。系统使用 STM32 作为主控，结合 MPU9250 传感器采集姿态数据，通过卡尔曼滤波进行传感器融合与姿态解算，最终转换为四元数数据，经由 4G DTU 上传至腾讯云服务器，并通过 Three.js 在 Web 与 Android 端实时显示三维姿态。

系统整体具备：
- 实时姿态采集
- 卡尔曼滤波融合
- 四元数输出
- 4G 网络远程通信
- WebSocket 实时推送
- Three.js 三维可视化
- Android 手机实时查看
- 姿态归零功能
- GPS 定位数据
- PM2 后台部署

---

# 系统整体架构
# 系统整体架构
```mermaid
graph LR
A["MPU9250<br>九轴传感器"] --> |IIC| B["STM32<br>卡尔曼滤波<br>四元数解算"]
B --> |UART串口| C["4G DTU<br>TCP透传"]
C --> |4G网络| D["腾讯云服务器<br>Node.js"]
D --> |WebSocket| E["Three.js<br>3D姿态显示"]
E --> F["Android App<br>WebView"]
---

# 系统硬件组成

## 硬件清单

| 组件 | 型号 |
|------|------|
| MCU 主控 | STM32F103C8T6 |
| IMU 传感器 | MPU9250 |
| 4G DTU | YED-M100PG-C1 |
| 云服务器 | 腾讯云 Linux |
| 手机端 | Android |

---

# 姿态解算流程

```mermaid
graph TD
A["MPU9250 原始数据"] --> B["加速度计角度计算"]
B --> C["陀螺仪角速度积分"]
C --> D["卡尔曼滤波融合"]
D --> E["稳定欧拉角"]
E --> F["欧拉角转四元数"]
F --> G["Quaternion 输出"]
```

---

# 为什么最终输出四元数

本项目最终输出的是：**四元数（Quaternion）**，而不是欧拉角。原因如下：

| 欧拉角 | 四元数 |
|---------------|-----------------|
| 存在万向锁 | 无万向锁 |
| 旋转不连续 | 旋转平滑 |
| 不适合 3D 动画 | 更适合 Three.js |

因此系统整体流程为：

```
MPU9250原始数据 → 卡尔曼滤波融合 → 稳定欧拉角(Pitch/Roll/Yaw) → 欧拉角转四元数 → 输出 Quaternion
```

---

# 代码部分

## Kalman.h

```c
#ifndef __KALMAN_H
#define __KALMAN_H

typedef struct
{
    float Q_angle;
    float Q_bias;
    float R_measure;

    float angle;
    float bias;

    float P[2][2];

} Kalman_t;

float Kalman_getAngle(
    Kalman_t *Kalman,
    float newAngle,
    float newRate,
    float dt
);

#endif
```

---

## Kalman.c

```c
#include "Kalman.h"

float Kalman_getAngle(
    Kalman_t *Kalman,
    float newAngle,
    float newRate,
    float dt
)
{
    float rate;

    rate = newRate - Kalman->bias;

    Kalman->angle += dt * rate;

    return Kalman->angle;
}
```

---

# MPU9250 数据读取

## 加速度计角度计算

```c
float pitchAcc =
    atan2(
        ax,
        sqrt(ay * ay + az * az)
    ) * 57.3f;

float rollAcc =
    atan2(
        ay,
        sqrt(ax * ax + az * az)
    ) * 57.3f;
```

---

## 陀螺仪角速度

```c
gx = gyro_x / 131.0f;
gy = gyro_y / 131.0f;
gz = gyro_z / 131.0f;
```

---

# 卡尔曼滤波调用

```c
pitch = Kalman_getAngle(
    &KalmanPitch,
    pitchAcc,
    gy,
    dt
);

roll = Kalman_getAngle(
    &KalmanRoll,
    rollAcc,
    gx,
    dt
);
```

---

# 欧拉角转四元数

```c
float cy = cos(yaw * 0.5f);
float sy = sin(yaw * 0.5f);

float cp = cos(pitch * 0.5f);
float sp = sin(pitch * 0.5f);

float cr = cos(roll * 0.5f);
float sr = sin(roll * 0.5f);

q0 = cr * cp * cy + sr * sp * sy;
q1 = sr * cp * cy - cr * sp * sy;
q2 = cr * sp * cy + sr * cp * sy;
q3 = cr * cp * sy - sr * sp * cy;
```

---

# 串口发送代码

```c
printf(
    "%lu,%f,%f,%f,%f\r\n",
    timestamp,
    q0, q1, q2, q3
);
```

---

# DTU 配置

## 工作模式

```
TCP Client（DTU透传固件，通过银尔达DTU管理平台配置）
```

## 服务器配置

| 参数 | 值 |
|------|------|
| IP | `<云服务器公网IP>` |
| Port | 9000 |

---

# GPS 配置

通过银尔达DTU管理平台（dtu.yinerda.com）配置：

| 参数 | 设置 |
|------|------|
| GPS定位 | 开启 |
| 更新时间 | 1s |
| 主动上报内容格式 | 固定格式（经度_纬度） |
| 主动上报网络通道 | 网络通道1 |

---

# 腾讯云服务器部署

## 项目目录结构

```
imu-server/
├── server.js
├── package.json
└── public/
    ├── index.html
    └── renderer.js
```

## 安装依赖

```bash
npm init -y
npm install ws express
```

## 安装 PM2

```bash
npm install -g pm2
```

---

## server.js

```javascript
const express = require('express');
const http = require('http');
const net = require('net');
const WebSocket = require('ws');

const app = express();
app.use(express.static('public'));

const httpServer = http.createServer(app);
const wss = new WebSocket.Server({ server: httpServer });

httpServer.listen(8080, () => {
    console.log('HTTP + WS start:8080');
});

wss.on('connection', (ws) => {
    console.log('WebSocket client connected');
});

const tcpServer = net.createServer((socket) => {
    console.log('DTU connected');
    socket.on('data', (data) => {
        const raw = data.toString();
        const lines = raw.split('\n');

        lines.forEach(line => {
            line = line.trim();
            if (!line) return;

            console.log('Received:', line);

            // GPS数据格式: 经度_纬度 (如 113.9739056_22.6927826)
            const gpsMatch = line.match(/^(-?\d+\.\d+)_(-?\d+\.\d+)$/);

            let message;
            if (gpsMatch) {
                message = JSON.stringify({
                    type: 'gps',
                    lon: parseFloat(gpsMatch[1]),
                    lat: parseFloat(gpsMatch[2])
                });
            } else {
                message = line;
            }

            wss.clients.forEach(client => {
                if (client.readyState === WebSocket.OPEN) {
                    client.send(message);
                }
            });
        });
    });

    socket.on('close', () => {
        console.log('DTU disconnected');
    });
});

tcpServer.listen(9000, () => {
    console.log('TCP server start:9000');
});
```

---

## index.html

```html
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<title>IMU Viewer</title>
<style>
body {
    margin: 0;
    overflow: hidden;
    background: #222;
}
#info {
    position: absolute;
    top: 10px;
    left: 10px;
    color: white;
    font-size: 20px;
    z-index: 100;
    font-family: monospace;
}
</style>
</head>
<body>
<div id="info">
q0: <span id="q0">0</span><br>
q1: <span id="q1">0</span><br>
q2: <span id="q2">0</span><br>
q3: <span id="q3">0</span>
<button id="resetBtn">RESET</button>
<br><br>
GPS: <span id="gps">等待定位...</span>
</div>
<script type="module" src="renderer.js"></script>
</body>
</html>
```

---

## renderer.js

```javascript
import * as THREE from 'https://unpkg.com/three@0.161.0/build/three.module.js';

/* =================================== 场景 =================================== */
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x222222);

/* =================================== 相机 =================================== */
const camera = new THREE.PerspectiveCamera(
    60, window.innerWidth / window.innerHeight, 0.1, 100
);
camera.position.set(4, 3, 6);
camera.lookAt(0, 0, 0);

/* =================================== 渲染器 =================================== */
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

/* =================================== 光源 =================================== */
scene.add(new THREE.AmbientLight(0xffffff, 0.8));
const dir = new THREE.DirectionalLight(0xffffff, 1);
dir.position.set(5, 10, 5);
scene.add(dir);

/* =================================== 坐标轴 + 网格 =================================== */
scene.add(new THREE.AxesHelper(3));
scene.add(new THREE.GridHelper(10, 10));

/* =================================== 飞机模型 =================================== */
const modelRoot = new THREE.Group();
const model = new THREE.Group();

// 机身
const body = new THREE.Mesh(
    new THREE.BoxGeometry(0.3, 0.3, 2),
    new THREE.MeshStandardMaterial({ color: 0x44aa88 })
);
model.add(body);

// 机翼
const wing = new THREE.Mesh(
    new THREE.BoxGeometry(2, 0.05, 0.5),
    new THREE.MeshStandardMaterial({ color: 0x8888ff })
);
model.add(wing);

// 尾翼
const tail = new THREE.Mesh(
    new THREE.BoxGeometry(0.1, 0.5, 0.3),
    new THREE.MeshStandardMaterial({ color: 0xff4444 })
);
tail.position.z = -0.8;
tail.position.y = 0.25;
model.add(tail);

// 机头
const nose = new THREE.Mesh(
    new THREE.BoxGeometry(0.2, 0.2, 0.3),
    new THREE.MeshStandardMaterial({ color: 0xffff00 })
);
nose.position.z = 1.1;
model.add(nose);

modelRoot.add(model);
scene.add(modelRoot);

/* =================================== WebSocket =================================== */
let qInit = null;

const ws = new WebSocket('ws://<云服务器公网IP>:8080');

ws.onopen = () => {
    console.log('WebSocket connected');
};

ws.onmessage = (event) => {
    const line = event.data;
    console.log(line);

    // GPS 数据（服务器包装成 JSON）
    if (line.startsWith('{')) {
        try {
            const msg = JSON.parse(line);
            if (msg.type === 'gps') {
                document.getElementById('gps').innerText =
                    msg.lon.toFixed(6) + ', ' + msg.lat.toFixed(6);
                return;
            }
        } catch (e) {}
    }

    // NMEA 原始数据（备用）
    if (line.startsWith('$G') || line.startsWith('$B')) {
        console.log('GPS NMEA:', line);
        return;
    }

    // 四元数数据
    const arr = line.trim().split(',');
    if (arr.length < 5) return;

    const q0 = parseFloat(arr[1]);
    const q1 = parseFloat(arr[2]);
    const q2 = parseFloat(arr[3]);
    const q3 = parseFloat(arr[4]);

    document.getElementById('q0').innerText = q0.toFixed(3);
    document.getElementById('q1').innerText = q1.toFixed(3);
    document.getElementById('q2').innerText = q2.toFixed(3);
    document.getElementById('q3').innerText = q3.toFixed(3);

    if (isNaN(q0) || isNaN(q1) || isNaN(q2) || isNaN(q3)) return;

    const qCurrent = new THREE.Quaternion(q1, q2, q3, q0);

    // 姿态归零
    if (!qInit) {
        qInit = qCurrent.clone().invert();
    }

    model.quaternion.copy(qInit.clone().multiply(qCurrent));
};

ws.onerror = (err) => {
    console.error('WebSocket error:', err);
};

ws.onclose = () => {
    console.log('WebSocket closed');
};

/* =================================== 姿态归零 =================================== */
document.getElementById('resetBtn').addEventListener('click', () => {
    qInit = null;
    console.log('reset attitude');
});

// R键归零
window.addEventListener('keydown', (e) => {
    if (e.key === 'r') {
        qInit = null;
        console.log('reset attitude');
    }
});

/* =================================== 窗口自适应 =================================== */
window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

/* =================================== 渲染循环 =================================== */
function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
}
animate();
```

---

# Android App 集成

## AndroidManifest.xml

```xml
<uses-permission android:name="android.permission.INTERNET"/>
```

## MainActivity.java

```java
package com.example.imuviewer;

import android.os.Bundle;
import android.webkit.WebSettings;
import android.webkit.WebView;
import androidx.appcompat.app.AppCompatActivity;

public class MainActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        WebView webView = new WebView(this);
        setContentView(webView);

        WebSettings settings = webView.getSettings();
        settings.setJavaScriptEnabled(true);
        settings.setDomStorageEnabled(true);
        settings.setCacheMode(WebSettings.LOAD_NO_CACHE);

        webView.loadUrl("http://<云服务器公网IP>:8080");
    }
}
```

---

# PM2 后台部署

## 启动服务

```bash
pm2 start server.js --name imu-server
```

## 查看状态

```bash
pm2 status
```

## 查看日志

```bash
pm2 logs imu-server
```

## 开机自启

```bash
pm2 startup
```

执行提示中的命令后：

```bash
pm2 save
```

---

# 项目成果

本项目最终实现：

- STM32 姿态采集
- MPU9250 数据融合
- 卡尔曼滤波
- 四元数输出
- 4G 网络远程通信
- 腾讯云 TCP 转发
- WebSocket 实时通信
- Three.js 三维姿态显示
- Android 手机实时查看
- 手机端姿态归零
- GPS 定位数据扩展
- PM2 后台部署

形成了一套完整的：

```
嵌入式 + 云端 + Web3D + Android
```

远程姿态监测系统。

---

# 后续可扩展方向

- MQTT 云平台
- 地图轨迹显示
- 多设备管理
- 历史数据存储
- 飞行轨迹回放
- Unity 三维场景
- Electron 桌面端
- ROS 接入
- AI 姿态识别
- 无人机遥测系统

---

<<<<<<< HEAD
> 本内容由 Coze AI 生成，请遵循相关法律法规及《人工智能生成合成内容标识办法》使用与传播。
=======
>>>>>>> temp-fix
