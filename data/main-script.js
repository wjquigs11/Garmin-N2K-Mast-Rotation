
// Get current sensor readings when the page loads  
window.addEventListener('load', getReadings);

var gaugeMagRot = new RadialGauge({
  renderTo: 'gauge-magrot',
  width: 300,
  height: 300,
//    units: "degrees",
  minValue: -50,
  maxValue: 50,
  startAngle: 310,
  ticksAngle: 100,
//    colorValueBoxRect: "#049faa",
//    colorValueBoxRectEnd: "#049faa",
//    colorValueBoxBackground: "#f1fbfc",
  valueInt: 2,
  valueBox: false,
  majorTicks: [
      "-50",
      "0",
      "50"
  ],
  minorTicks: 4,
  strokeTicks: true,
  highlights: [
      {
          "from": -50,
          "to": 50,
          "color": "#D3D3D3"
      }
  ],
  colorPlate: "#fff",
  borderShadowWidth: 0,
  borders: false,
  needleType: "arrow",
  colorNeedle: "#0A1128",
  colorNeedleEnd: "#D3D3D3",
  needleWidth: 3,
  needleCircleSize: 5,
  colorNeedleCircleOuter: "#D3D3D3",
  needleCircleOuter: true,
  needleCircleInner: false,
  animationDuration: 10,
  animationRule: "linear",
  title: "Magnetic"
}).draw();

var gaugeHoneyRot = new RadialGauge({
  renderTo: 'gauge-honeyrot',
  width: 300,
  height: 300,
//    units: "degrees",
  minValue: -50,
  maxValue: 50,
  startAngle: 310,
  ticksAngle: 100,
//    colorValueBoxRect: "#049faa",
//    colorValueBoxRectEnd: "#049faa",
//    colorValueBoxBackground: "#f1fbfc",
  valueInt: 2,
  valueBox: false,
  majorTicks: [
      "-50",
      "0",
      "50"
  ],
  minorTicks: 4,
  strokeTicks: true,
  highlights: [
      {
          "from": -50,
          "to": 50,
          "color": "#D3D3D3"
      }
  ],
  colorPlate: "#fff",
  borderShadowWidth: 0,
  borders: false,
  needleType: "arrow",
  colorNeedle: "#0A1128",
  colorNeedleEnd: "#D3D3D3",
  needleWidth: 3,
  needleCircleSize: 5,
  colorNeedleCircleOuter: "#D3D3D3",
  needleCircleOuter: true,
  needleCircleInner: false,
  animationDuration: 10,
  animationRule: "linear",
  title: "Sensor"
}).draw();

// Function to get current readings on the webpage when it loads for the first time
// do we really need this, since our update rate is at least 10Hz?
function getReadings() {
    var xhr = new XMLHttpRequest();
    xhr.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        var myObj = JSON.parse(this.responseText);
        gaugeMagRot.value = -myObj.mastDelta;
        gaugeHoneyRot.value = myObj.mastRotate;
        document.getElementById('mastRotate').innerHTML = myObj.mastRotate;
        document.getElementById('windSpeed').innerHTML = myObj.windSpeed;
        document.getElementById('windAngle').innerHTML = myObj.windAngle;      
      }
    }; 
    xhr.open("GET", "/readings", true);
    xhr.send();
  }
  
  if (!!window.EventSource) {
    var source = new EventSource('/events');
    
    source.addEventListener('open', function(e) {
      console.log("Events Connected");
    }, false);
  
    source.addEventListener('error', function(e) {
      if (e.target.readyState != EventSource.OPEN) {
        console.log("Events Disconnected");
      }
    }, false);
    
    source.addEventListener('message', function(e) {
      console.log("message", e.data);
    }, false);
    
    source.addEventListener('new_readings', function(e) {
      console.log("new_readings", e.data);
      var myObj = JSON.parse(e.data);
      console.log(myObj);
      //updatePage(myObj);
      gaugeMagRot.value = -myObj.mastDelta;
      gaugeHoneyRot.value = myObj.mastRotate;
      document.getElementById('mastRotate').innerHTML = myObj.mastRotate;
      document.getElementById('windSpeed').innerHTML = myObj.windSpeed;
      document.getElementById('windAngle').innerHTML = myObj.windAngle;
      document.getElementById('mastHeading').innerHTML = myObj.mastHeading.toFixed(2);
      document.getElementById('compassHeading').innerHTML = myObj.compassHeading;
      document.getElementById('mastDelta').innerHTML = myObj.mastDelta;
      document.getElementById('boatTrue').innerHTML = myObj.boatTrue;
      document.getElementById('rotateout').innerHTML = myObj.rotateout;
    }, false);
  }