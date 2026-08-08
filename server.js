const http = require('http');
const express = require('express');
const app = express();

app.use(express.static('public'));

const fs = require('fs');
const path = require('path');
const net = require('net');
const WebSocket = require('ws');
const server = http.createServer(app);

const wss = new WebSocket.Server({ server });

wss.on('connection', (ws) => {
    console.log("WebSocket client connected");
});

const tcpServer = net.createServer((socket) => {
    console.log("DTU connected");
    socket.on("data", (data) => {
        const raw = data.toString().trim();
        console.log("Received:", raw);

        // GPS数据格式: 经度_纬度 (如 113.9739056_22.6927826)
        const gpsMatch = raw.match(/^(-?\d+\.\d+)_(-?\d+\.\d+)$/);

        let message;
        if (gpsMatch) {
            // GPS数据，包装成JSON发给App
            message = JSON.stringify({
                type: 'gps',
                lon: parseFloat(gpsMatch[1]),
                lat: parseFloat(gpsMatch[2])
            });
        } else {
            // 姿态数据，保持原样转发
            message = raw;
        }

        wss.clients.forEach(client => {
            if (client.readyState === WebSocket.OPEN) {
                client.send(message);
            }
        });
    });
});

server.listen(8080, () => {
    console.log("HTTP + WS start:8080");
});

tcpServer.listen(9000, () => {
    console.log("TCP server start:9000");
});
