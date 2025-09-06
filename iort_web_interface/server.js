const WebSocket = require('ws');

// Start WebSocket server on port 8080
const wss = new WebSocket.Server({ host: '0.0.0.0', port: 8080 });

console.log("WebSocket server running on ws://192.168.1.9:8080");

wss.on('connection', (ws) => {
  console.log("New client connected");

  // Send a welcome message
  ws.send("Hello from WebSocket server!");

  // Handle messages from clients
  ws.on('message', (message) => {
    console.log("Received:", message);

    // Echo the message back to all clients
    wss.clients.forEach((client) => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(message);
      }
    });
  });

  ws.on('close', () => {
    console.log("Client disconnected");
  });
});
