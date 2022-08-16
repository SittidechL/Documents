// Complete Project Details at: https://RandomNerdTutorials.com/

// Database Paths
var dataFloatPath = 'test/float';
var dataIntPath = 'test/int';
var dataButtonPath = 'test/button';
var datamyValuePath = 'test/myValue';
var dataLDRPath = 'topic/ldr';

// Get a database reference
const databaseFloat = database.ref(dataFloatPath);
const databaseInt = database.ref(dataIntPath);
const databaseButton = database.ref(dataButtonPath);
const databasemyValue = database.ref(datamyValuePath);
const databaseldrValue = database.ref(dataLDRPath);

// Variables to save database current values
var floatReading;
var intReading;
var buttonReading;
var myValueReading;
var ldrReading;

var index = 0;
var xValues = [''];
var yValues = [''];


// Attach an asynchronous callback to read the data
databaseFloat.on('value', (snapshot) => {
  floatReading = snapshot.val();
  console.log(floatReading);
  document.getElementById("reading-float").innerHTML = floatReading;
  gauge.refresh(floatReading);
  yValues[index] = floatReading;

}, (errorObject) => {
  console.log('The read failed: ' + errorObject.name);
});

databaseInt.on('value', (snapshot) => {
  intReading = snapshot.val();
  console.log(intReading);
  document.getElementById("reading-int").innerHTML = intReading;
  if (intReading != null)
  xValues[index] = intReading;
  chart.update()
  index += 1;
}, (errorObject) => {
  console.log('The read failed: ' + errorObject.name);
});

databaseButton.on('value', (snapshot) => {
    buttonReading = snapshot.val();
    console.log(buttonReading);
    document.getElementById("reading-button").innerHTML = buttonReading;
  }, (errorObject) => {
    console.log('The read failed: ' + errorObject.name);
  });

  databasemyValue.on('value', (snapshot) => {
    myValueReading = snapshot.val();
    console.log(myValueReading);
    document.getElementById("reading-myValue").innerHTML = myValueReading;
  }, (errorObject) => {
    console.log('The read failed: ' + errorObject.name);
  });

  databaseldrValue.on('value', (snapshot) => {
    ldrReading = snapshot.val();
    console.log(ldrReading);
    document.getElementById("reading-ldr").innerHTML = ldrReading;
  }, (errorObject) => {
    console.log('The read failed: ' + errorObject.name);
  });


  gauge = new JustGage({
    id: 'gauge',
    value: 0,
    min: 0,
    max: 100,
    symbol: '',
    pointer: true,
    gaugeWidthScale: 0.6,
    counter: true,
    relativeGaugeSize: true,
    donut: true
  });

var dataPathMessage = 'web/'
const databaseMessage = database.ref(dataPathMessage);
function sendValue() {
    var message = document.getElementById("inputMessage").value;
    if(message != 0 && message != 1)
        alert("กรุณาป้อน 0 หรือ 1");
    else
        databaseMessage.set({ message });
}

chart = new Chart("myChart", {
  type: "line",
  data: {
    labels: xValues,
    datasets: [{
      fill: false,
      lineTension: 0,
      backgroundColor: "rgba(0,0,255,1.0)",
      borderColor: "rgba(0,0,255,0.1)",
      data: yValues
    }]
  },
  options: {
    legend: { display: false },
    scales: {
      yAxes: [{ ticks: { min: 0, max: 100 } }],
    }
  }
}); 

// Create Temperature Gauge
function createTemperatureGauge() {
  var gauge = new LinearGauge({
      renderTo: 'gauge-temperature',
      width: 120,
      height: 400,
      units: "Temperature C",
      minValue: 0,
      startAngle: 90,
      ticksAngle: 180,
      maxValue: 40,
      colorValueBoxRect: "#049faa",
      colorValueBoxRectEnd: "#049faa",
      colorValueBoxBackground: "#f1fbfc",
      valueDec: 2,
      valueInt: 2,
      majorTicks: [
          "0",
          "5",
          "10",
          "15",
          "20",
          "25",
          "30",
          "35",
          "40"
      ],
      minorTicks: 4,
      strokeTicks: true,
      highlights: [
          {
              "from": 30,
              "to": 40,
              "color": "rgba(200, 50, 50, .75)"
          }
      ],
      colorPlate: "#fff",
      colorBarProgress: "#CC2936",
      colorBarProgressEnd: "#049faa",
      borderShadowWidth: 0,
      borders: false,
      needleType: "arrow",
      needleWidth: 2,
      needleCircleSize: 7,
      needleCircleOuter: true,
      needleCircleInner: false,
      animationDuration: 1500,
      animationRule: "linear",
      barWidth: 10,
  });
  return gauge;
}
