const char htmlPage9[] PROGMEM = R"=====(
<html>
<head>
    <meta http-equiv="cache-control" content="max-age=0" />
    <meta http-equiv="cache-control" content="no-cache" />
    <meta http-equiv="expires" content="0" />
    <meta http-equiv="expires" content="Tue, 01 Jan 1980 1:00:00 GMT" />
    <meta http-equiv="pragma" content="no-cache" />
    <meta http-equiv="content-type" content="text/html;charset=UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <script src="https://ajax.googleapis.com/ajax/libs/jquery/3.3.1/jquery.min.js"></script>
    <script src="https://maxcdn.bootstrapcdn.com/bootstrap/3.4.1/js/bootstrap.min.js"></script>
    <title>Ardu ECU - Start &amp; Throttle</title>

<style>
font-family: Arial, Helvetica, sans-serif; 
  text-align: center;

h1 {
  font-size: 1.8rem; 
  color: white;
   margin: 5%;
}

p { 
  align-items: center;
  padding-left: 5%;
}
form {
  margin-left: 5%;
}
span{
 margin-left: 5%;
}
.topnav { 
  overflow: hidden; 
  background-color: #0A1128;
}

.card-grid { 
  max-width: 1200px; 
  margin: 0 auto; 
  display: grid; 
  grid-gap: 2rem; 
  grid-template-columns: repeat(auto-fit, minmax(100px, 1fr));
}
.card { 
  background-color: white; 
  align-items: center;
  text-align: center;
  box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);
}
.card-title { 
  align-items: center;
  text-align: center;
  font-size: 1.2rem;
  font-weight: bold;
  color: #034078
}
.button {
  background-color: #1c87c9;
  border: none;
  color: white;
  padding: 14px 28px;
  text-align: center;
  text-decoration: none;
  display: inline-block;
  font-size: 20px;
  margin: 4px 2px;
  cursor: pointer;
  border-radius: 8px;
}
.bigbutton {
  padding: 18px 36px;
  font-size: 24px;
}
.startbutton { background-color: #1c8738; }
.stopbutton  { background-color: #c9281c; }
.glowbutton  { background-color: #c9821c; }
.rcbutton    { background-color: #1c87c9; }
input[type=range] { width: 90%; }
#thrval { font-size: 2rem; font-weight: bold; }
</style>
</head>

<body>
    <div class="container">
        <h1>Ardu_ECU</h1>
        <p>Engine Start &amp; Throttle Control</p>
        <style>
      .navbutton {
        background-color: #1c87c9;
        border: none;
        color: white;
        padding: 5px 9px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 16px;
        margin: 4px 2px;
        cursor: pointer;
        border-radius: 8px;
      }
    </style>
        <p>
           <a  href="/" class="navbutton">Page1</a>
            <a href="/page2" class="navbutton" >Page2</a>
            <a  href="/page3" class="navbutton">Page3</a>
            <a href="/page4" class="navbutton">Page4</a>
            <a  href="/page5" class="navbutton">Page5</a>
            <a  href="/page6" class="navbutton">Page6</a>
            <a  href="/page7" class="navbutton">Page7</a>
            <a  href="/page8" class="navbutton">Page8</a>
            <a  href="/page9" class="navbutton">Page9</a>
        </p>

    
    </div>
    <hr/>
   <div class="content">
     <div class="card-grid">
       <div class="card">
       <p class="card-title">Throttle Command</p><span id="thrval">0%</span>
       <input type="range" min="0" max="100" value="0" id="throttle" oninput="sendThrottle()">
       <p id="thrnote"></p>
       <a class="button startbutton" href="/start">START</a>
       <a class="button stopbutton" href="/stop">STOP</a>
       <a class="button glowbutton" href="/abortweb">ABORT</a>
       <a class="button rcbutton" href="/webRC">Use RC Control</a>
       <a class="button" href="/ResetError">Clear Error</a>
       </div>
       <div class="card">
       <p class="card-title">RPM</p><span id="rpm">-</span>
       <p class="card-title">Temperature</p><span id="temp">-</span>
       <p class="card-title">Mode</p><span id="mode">-</span>
       </div>
       <div class="card">
       <p class="card-title">Starter</p><span id="starter">-</span>
       <p class="card-title">Glow</p><span id="glow">-</span>
       <p class="card-title">Gas</p><span id="gas">-</span>
       </div>
       <div class="card">
       <p class="card-title">Fuel</p><span id="fuel">-</span>
       <p class="card-title">Batt.Volt</p><span id="batvolt">-</span>
       <p class="card-title">Error</p><span id="error">-</span>
       </div>
     </div>
    <script>
  sendkeepalive(); // get initial data straight away 
  
    // request data updates every 2000 milliseconds
    setInterval(sendkeepalive, 2000);
    setInterval(getReadings, 2000);

    function sendkeepalive() {
          var xhr2 = new XMLHttpRequest();
      xhr2.open("GET", "/webkeepalive");
      xhr2.send();
    }

// Send the throttle slider value to the ECU
function sendThrottle(){
  var t = document.getElementById("throttle").value;
  document.getElementById("thrval").innerHTML = t+"%";
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/throttle?t="+t, true);
  xhr.send();
}

// Function to get current readings on the webpage when it loads for the first time
function getReadings(){
  var xhr = new XMLHttpRequest();
  xhr.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var myObj = JSON.parse(this.responseText);
      var rpm = myObj.rpm;
      var throttle = myObj.throttle;
       var starter = myObj.starter;
        var glow = myObj.glow;
       var fuel = myObj.fuel;
        var gas = myObj.gas;
         var ecumode = myObj.ecumode;
          var error = myObj.error;
          var batvolt = myObj.batvolt;
      document.getElementById("rpm").innerHTML = 1000*rpm;
      document.getElementById("temp").innerHTML = myObj.temperature;
      document.getElementById("starter").innerHTML= starter;
      document.getElementById("glow").innerHTML = glow;
      document.getElementById("fuel").innerHTML = fuel;
      document.getElementById("gas").innerHTML=gas;
      document.getElementById("mode").innerHTML=ecumode;
      document.getElementById("error").innerHTML=error;
      document.getElementById("batvolt").innerHTML=batvolt;
      var slider = document.getElementById("throttle");
      if ((throttle>=0)&&(throttle<=100)) {
        slider.value = throttle;
        document.getElementById("thrval").innerHTML = throttle+"%";
      }
    }
  }; 
  xhr.open("GET", "/readings", true);
  xhr.send();
}
  </script>
            
  </body>

</html>
)=====";