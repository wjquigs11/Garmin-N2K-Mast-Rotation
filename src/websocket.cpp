#ifdef WSCLIENT
#include <WebSocketClient.h>
using namespace net;
WebSocketClient client;

void setupWS() {
  client.onOpen([](WebSocket &ws) {
    Serial.println(F("WS Connected"));
    const auto protocol = ws.getProtocol();
    if (protocol) {
      Serial.print(F("WS Client protocol: "));
      Serial.println(protocol);
    }

    const char message[]{"Hello from Arduino client!"};
    ws.send(WebSocket::DataType::TEXT, message, strlen(message));
  });

  client.onMessage([](WebSocket &ws, const WebSocket::DataType dataType,
                     const char *message, uint16_t length) {
    switch (dataType) {
    case WebSocket::DataType::TEXT:
      Serial.print(F("WS Received: "));
      Serial.println(message);
      break;
    case WebSocket::DataType::BINARY:
      Serial.println(F("WS Received binary data"));
      break;
    }

    //ws.send(dataType, message, length); // Echo back to server
  });

  client.onClose([](WebSocket &, const WebSocket::CloseCode, const char *,
                   uint16_t) { Serial.println(F("WS Disconnected\n")); });

  if (!client.open("mastcomp.local", 80, "/ws")) {
    Serial.println(F("WS Connection failed!"));
  }
}

void loopWS() {
  client.listen(); 
}

float getMastHeading() {
  return 0.0;
}
#endif
