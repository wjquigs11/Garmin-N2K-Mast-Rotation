
// Get current sensor readings when the page loads  
window.addEventListener('load', getReadings);
/*
var gaugeMagRot = new RadialGauge({
    renderTo: 'gauge-magrot',
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
    valueInt: 2,
    valueBox: true,
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

  var gaugeHoneyRot = new RadialGauge({
    renderTo: 'gauge-honeyrot',
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
    valueInt: 2,
    valueBox: true,
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
*/
function updatePage(myObj) {
  //var myObj = JSON.parse(gotUpdate.responseText);
  var keys = Object.keys(myObj);
  //console.log(keys);
  //console.log(keys.length);
  for (var i = 0; i < keys.length; i++) {
    var key = keys[i];
    console.log(key);
    // not sure if this is going to work. I'm trying to put the value of PotValue into the hidden input in the form to pass to the POST
    if (key == "PotValue") {
      document.getElementById("PotValuePass").textContent = myObj[key];
      console.log(myObj[key]);
      continue;
    }
    /*
    if (key == "mastDelta") {
      gaugeMagRot.value = myObj[key];
      console.log(myObj[key]);
      continue
    }*/
    var theElement = document.getElementById(key);
    if (theElement)
      theElement.textContent = myObj[key];
  }    
}

// Function to get current readings on the webpage when it loads for the first time
function getReadings() {
    var xhr = new XMLHttpRequest();
    xhr.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        var myObj = JSON.parse(this.responseText);
        updatePage(myObj);
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
      updatePage(myObj);
      //gaugeRot.value = myObj.temperature;
    }, false);
  }