
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
      //console.log(document.getElementById("honeywell"));
      //console.log(document.querySelector("#honeywell"));
      //console.log([...document.querySelectorAll('[id]')].map(el => el.id));
      if (myObj.mastRotate) // poor choice but mastRotate represents Honeywell sensor
        document.getElementById("honeywell").style.display = 'block';
      else
        document.getElementById("honeywell").style.display = 'none';
      //var gaugeCanvas = document.querySelector('#gauge-magrot');
      var gaugeCanvas = document.getElementById('gauge-magrot')
      if (gaugeCanvas) {
        if (myObj.compass) {
          // change the canvas element holding the gauge
          gaugeCanvas.removeAttribute("hidden");
          gaugeCanvas.style.display = '';
          // change the gauge itself
          gaugeMagRot.value = myObj.mastDelta;
        } else {
          gaugeCanvas.style.display = 'none';
        }
      }
      gaugeCanvas = document.getElementById('gauge-honeyrot');
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
      if (elementThis = document.getElementById('mastHeading')) elementThis.innerHTML = myObj.mastHeading;
      if (elementThis = document.getElementById('boatHeading')) elementThis.innerHTML = myObj.boatHeading;
      if (elementThis = document.getElementById('boatTrue')) elementThis.innerHTML = myObj.boatTrue;
      if (elementThis = document.getElementById('rotateout')) elementThis.innerHTML = myObj.rotateout;
      if (elementThis = document.getElementById('boatCalStatus')) {
        elementThis.innerHTML = myObj.boatCalStatus;
        let imageElement = document.getElementById("calibration");
        console.log("imageElement: ", imageElement);
        if (imageElement) {
          console.log("calstatus: ", myObj.boatCalStatus);
          switch (myObj.boatCalStatus) {
            case 0:
              imageElement.src = "red.png";
              break;
            case 1:
              imageElement.src = "orange.png";
              break;
            case 2:
              imageElement.src = "yellow.png";
              break;
            case 3:
              imageElement.src = "green.png";
              break;
            default:
              imageElement.src = "";
              break;
          }
          console.log(imageElement.src);
        } else {
          console.log("no calibration image");
        }     
      }
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