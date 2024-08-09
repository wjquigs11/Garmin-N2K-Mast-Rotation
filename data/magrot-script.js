
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
  valueBox: true,
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
