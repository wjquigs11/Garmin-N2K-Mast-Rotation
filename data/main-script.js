
// Get current sensor readings when the page loads  
window.addEventListener('load', getReadings);

// Function to get current readings on the webpage when it loads for the first time
// do we really need this, since our update rate is at least 10Hz?
function getReadings() {
  console.log("getReadings");
  var xhr = new XMLHttpRequest();
  xhr.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var myObj = JSON.parse(this.responseText);
      //gaugeMagRot.value = -myObj.mastDelta;
      //gaugeHoneyRot.value = myObj.mastRotate;
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
      var gaugeCanvas = document.querySelector('#gauge-magrot');
      if (gaugeCanvas) {
        if (myObj.compass) {
          gaugeCanvas.removeAttribute("hidden");
          gaugeCanvas.style.display = '';
          gaugeCanvas.value = -myObj.mastDelta;
        } else {
          gaugeCanvas.style.display = 'none';
        }
      }
      gaugeCanvas = document.querySelector('#gauge-honeyrot');
      if (gaugeCanvas) {
        if (myObj.honeywell) {
          gaugeCanvas.removeAttribute("hidden");
          gaugeCanvas.style.display = '';
          gaugeHoneyRot.value = myObj.mastRotate;
        } else { 
          gaugeCanvas.style.display = 'none';
        }
      }
      var elementThis;
      if (elementThis = document.getElementById('mastRotate')) elementThis.innerHTML = myObj.mastRotate;
      if (elementThis = document.getElementById('windSpeed')) elementThis.innerHTML = myObj.windSpeed;
      if (elementThis = document.getElementById('windAngle')) elementThis.innerHTML = myObj.windAngle;
      if (elementThis = document.getElementById('mastHeading')) elementThis.innerHTML = myObj.mastHeading.toFixed(2);
      //} else document.getElementById('mastHeading') elementThis.innerHTML = 'undefined';
      if (elementThis = document.getElementById('compassHeading')) elementThis.innerHTML = myObj.compassHeading.toFixed(2);
      if (elementThis = document.getElementById('mastDelta')) elementThis.innerHTML = myObj.mastDelta;
      if (elementThis = document.getElementById('boatTrue')) elementThis.innerHTML = myObj.boatTrue.toFixed(2);
      if (elementThis = document.getElementById('rotateout')) elementThis.innerHTML = myObj.rotateout;
    }, false);
  }

  function menuFun() {
    var x = document.getElementById("myLinks");
    if (x.style.display === "block") {
      x.style.display = "none";
    } else {
      x.style.display = "block";
    }
  }