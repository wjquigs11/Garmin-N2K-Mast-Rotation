window.addEventListener("load", getReadings);

function getReadings() {
    console.log("getReadings");
    var xhr = new XMLHttpRequest;
    xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            var myObj = JSON.parse(this.responseText);
            document.getElementById("mastRotate").innerHTML = myObj.mastRotate;
            document.getElementById("windSpeed").innerHTML = myObj.windSpeed;
            document.getElementById("windAngle").innerHTML = myObj.windAngle
        }
    };
    xhr.open("GET", "/readings", true);
    xhr.send()
}
if (!!window.EventSource) {
    var source = new EventSource("/events");
    source.addEventListener("open", function(e) {
        console.log("Events Connected")
    }, false);
    source.addEventListener("error", function(e) {
        if (e.target.readyState != EventSource.OPEN) {
            console.log("Events Disconnected")
        }
    }, false);
    source.addEventListener("message", function(e) {
        console.log("message", e.data)
    }, false);
    source.addEventListener("new_readings", function(e) {
        console.log("new_readings", e.data);
        var myObj = JSON.parse(e.data);
        var Honeywell = document.getElementById("honeywell");
        if (Honeywell) {
            if (myObj.mastRotate) Honeywell.style.display = "block";
            else Honeywell.style.display = "none";
        }
        var gaugeCanvas = document.getElementById("gauge-magrot");
        if (gaugeCanvas) {
            if (myObj.compass) {
                console.log("showing compass: ", myObj.compass);
                gaugeCanvas.removeAttribute("hidden");
                gaugeCanvas.style.display = "";
                gaugeMagRot.value = myObj.mastDelta
            } else {
                gaugeCanvas.style.display = "none"
            }
        }
        gaugeCanvas = document.getElementById("gauge-honeyrot");
        if (gaugeCanvas) {
            if (myObj.honeywell) {
                gaugeCanvas.removeAttribute("hidden");
                gaugeCanvas.style.display = "";
                gaugeHoneyRot.value = myObj.mastRotate
            } else {
                gaugeCanvas.style.display = "none"
            }
        }
        var elementThis;
        if (elementThis = document.getElementById("mastRotate")) elementThis.innerHTML = myObj.mastRotate;
        if (elementThis = document.getElementById("windSpeed")) elementThis.innerHTML = myObj.windSpeed;
        if (elementThis = document.getElementById("windAngle")) elementThis.innerHTML = myObj.windAngle;
        if (elementThis = document.getElementById("mastHeading")) elementThis.innerHTML = myObj.mastHeading;
        if (elementThis = document.getElementById("boatHeading")) elementThis.innerHTML = myObj.boatHeading;
        if (elementThis = document.getElementById("boatHeadingPi")) elementThis.innerHTML = myObj.boatHeadingPi;
        if (elementThis = document.getElementById("boatTrue")) elementThis.innerHTML = myObj.boatTrue;
        if (elementThis = document.getElementById("rotateout")) elementThis.innerHTML = myObj.rotateout;
        if (elementThis = document.getElementById("calibration")) {
            elementThis.innerHTML = myObj.boatCalStatus;
            if (elementThis) {
                console.log("calstatus: ", myObj.boatCalStatus);
                switch (myObj.boatCalStatus) {
                    case "0":
                        elementThis.src = "red.png";
                        break;
                    case "1":
                        elementThis.src = "orange.png";
                        break;
                    case "2":
                        elementThis.src = "yellow.png";
                        break;
                    case "3":
                        elementThis.src = "green.png";
                        break;
                    default:
                        elementThis.src = "";
                        break
                }
                console.log(elementThis.src)
            } else {
                console.log("no calibration image")
            }
        }
    }, false)
}

function menuFun() {
    var x = document.getElementById("myLinks");
    if (x.style.display === "block") {
        x.style.display = "none"
    } else {
        x.style.display = "block"
    }
}