
var gateway = `ws://${window.location.hostname}/ws`;
var websocket;
// Init web socket when the page loads
window.addEventListener('load', onload);

function onload(event) {
    initWebSocket();
}

function getReadings(){
    websocket.send("getReadings");
}

function initWebSocket() {
    console.log('Trying to open a WebSocket connection…');
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
}

// When websocket is established, call the getReadings() function
function onOpen(event) {
    console.log('Connection opened');
    getReadings();
}

function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}

// Function that receives the message from the ESP32 with the readings
function onMessage(event) {
    console.log(event.data);
    var myObj = JSON.parse(event.data);
    var keys = Object.keys(myObj);

    for (var i = 0; i < keys.length; i++){
        var key = keys[i];
        document.getElementById(key).innerHTML = myObj[key];
    }
}
/*
var gaugeRotX = new RadialGauge({
    renderTo: 'gauge-rotationX',
    width: 300,
    height: 300,
    units: "degrees",
    minValue: 0,
    startAngle: 310,
    ticksAngle: 100,
    valueBox: false,
    maxValue: 50,
    majorTicks: [
        "50",
        "40",
        "30",
        "20",
        "10",
        "0",
        "10",
        "20",
        "30",
        "40",
        "50"
    ],
    minorTicks: 2,
    strokeTicks: true,
    colorPlate: "#fff",
    borderShadowWidth: 0,
    borders: false,
    needleType: "line",
    needleWidth: 2,
    needleCircleSize: 1,
    needleCircleOuter: true,
    needleCircleInner: false,
    animationDuration: 1500,
    animationRule: "linear"
}).draw();
*/

var gaugeRot = new RadialGauge({
    renderTo: 'gauge-rotation',
    width: 300,
    height: 300,
//    units: "degrees",
    minValue: 0,
    maxValue: 100,
    startAngle: 310,
    ticksAngle: 100,
    colorValueBoxRect: "#049faa",
    colorValueBoxRectEnd: "#049faa",
    colorValueBoxBackground: "#f1fbfc",
//    valueInt: 2,
    valueBox: false,
    majorTicks: [
        "50",
        "0",
        "50"
    ],
    minorTicks: 4,
    strokeTicks: true,
    highlights: [
        {
            "from": 0,
            "to": 100,
            "color": "#03C0C1"
        }
    ],
    colorPlate: "#fff",
    borderShadowWidth: 0,
    borders: false,
    needleType: "arrow",
    colorNeedle: "#007F80",
    colorNeedleEnd: "#007F80",
    needleWidth: 3,
    needleCircleSize: 5,
    colorNeedleCircleOuter: "#007F80",
    needleCircleOuter: true,
    needleCircleInner: false,
    animationDuration: 1500,
    animationRule: "linear"
  }).draw();