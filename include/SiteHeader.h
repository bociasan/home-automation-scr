#include <Arduino.h>

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Home</title>
</head>
<style>
    html {
        font-family: Arial, Helvetica, sans-serif;
        text-align: center;
    }

    body {
        margin: 0;
    }

    h1 {
        font-size: 1.8rem;
        color: white;
    }

    .topnav {
        overflow: hidden;
        background-color: #143642;
    }

    .data {
        padding: 30px;
        max-width: 600px;
        margin: 0 auto;
        /*width: 300px;*/
    }

    button{
        margin: 10px 2px;

        min-width: 125px;
        padding: 13px 30px;
        font-size: 18px;
        text-align: center;
        outline: none;
        color: #fff;
        background-color: #0f8b8d;
        border: none;
        border-radius: 5px;
        -webkit-touch-callout: none;
        -webkit-user-select: none;
        -khtml-user-select: none;
        -moz-user-select: none;
        -ms-user-select: none;
        user-select: none;
        -webkit-tap-highlight-color: rgba(0,0,0,0);
    }
    button:active:enabled {
        background-color: #0f8b8d;
        box-shadow: 2px 2px #CDCDCD;
        transform: translateY(2px);
    }
    button:hover {background-color: #0f8baf}

    button:disabled, button[disabled]{
        border: 1px solid #999999;
        background-color: #cccccc;
        color: #666666;
    }

    .container-with-border {
        width: inherit;
        /*border: 1px solid black;*/
        border-radius: 10px;
        padding: 15px;
        margin: 10px 0;

        background-color: #F8F7F9;;
        box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);
    }

    .container-title{
        font-weight: bold;
        font-size: 23px;
    }

    .container-subtitle{
        font-weight: bold;
        font-style: italic;
        font-size: 15px;
    }
</style>
<body>
<div class="topnav">
    <h1>Smart House</h1>
</div>
<div class="data">
    <div class="container-with-border">
        <div class="container-title">Status</div>
        <p> Temperature: <span id="temperature">--</span></p>
        <p> Humidity: <span id="humidity">--</span></p>
        <p> Door status: <span id="isLocked">--</span></p>
        <p> Smoke status: <span id="gasValue">--</span></p>
        <p> Luminosity status: <span id="isNIGHT">--</span></p>
        <p> Light automation: <span id="lightAutomation">--</span></p>
        <p> Indoor light: <span id="ledIn">&#45;&#45;</span></p>
        <p> Outdoor light: <span id="ledOut">&#45;&#45;</span></p>
    </div>

    <div class="container-with-border">

        <div class="container-title">Door</div>
        <button id="lock">Lock</button>
        <button id="unlock">Unlock</button>
    </div>
        <div class="container-with-border">
            <div class="container-title">Light</div>
        <button id="auto">Auto</button>
        <button id="manual">Manual</button>

        <div id="panel-button-led-out">
            <div class="container-subtitle"> Indoor light: </div>

            <button id="button-led-in-on">On</button>
            <button id="button-led-in-off">OFF</button>
        </div>



        <div id="panel-button-led-in">
            <div class="container-subtitle"> Outdoor light: </div>
            <button id="button-led-out-on">On</button>
            <button id="button-led-out-off">OFF</button>
        </div>
        </div>
</div>

<script>
    window.addEventListener('load', onLoad);
    (function attachOnclick() {
        document.getElementById("lock").onclick = () => ws.send('lockDoor');
        document.getElementById("unlock").onclick = () => ws.send('unlockDoor');
        document.getElementById("auto").onclick = () => ws.send('automationOn');
        document.getElementById("manual").onclick = () => ws.send('automationOff');

        document.getElementById("button-led-in-on").onclick = () => ws.send('turnLEDinON');
        document.getElementById("button-led-in-off").onclick = () => ws.send('turnLEDinOFF');
        document.getElementById("button-led-out-on").onclick = () => ws.send('turnLEDoutON');
        document.getElementById("button-led-out-off").onclick = () => ws.send('turnLEDoutOFF');
    })()

    function updateState(state) {
        document.getElementById('temperature').innerText = state.temperatureValue + '\u2103'
        document.getElementById('humidity').innerText = state.humidityValue + "%"
        document.getElementById('isLocked').innerText = state.isLocked ? 'locked' : 'unlocked'
        document.getElementById('gasValue').innerText = state.gasValue ? 'no' : 'detected'
        document.getElementById('isNIGHT').innerText = state.isNIGHT ? 'night' : 'day'
        document.getElementById('lightAutomation').innerText = state.lightAutomation ? 'auto' : 'manual'
        document.getElementById('ledIn').innerText = state.ledInVal ? 'on' : 'off'
        document.getElementById('ledOut').innerText = state.ledOutVal ? 'on' : 'off'

        // document.getElementById("panel-button-led-out").style.visibility = state.lightAutomation ? "hidden" : "visible"
        // document.getElementById("panel-button-led-in").style.visibility = state.lightAutomation ? "hidden" : "visible"

        document.getElementById("button-led-out-on").disabled = state.lightAutomation
        document.getElementById("button-led-out-off").disabled = state.lightAutomation
        document.getElementById("button-led-in-on").disabled = state.lightAutomation
        document.getElementById("button-led-in-off").disabled = state.lightAutomation

    }

    let ws;

    function onLoad(event) {
        initWebSocket();
    }

    function initWebSocket() {
        console.log('Trying to open a WebSocket connection...');
        ws = new WebSocket(`ws://${window.location.hostname}/ws`);
        
        ws.onmessage = (webSocketMessage) => {
            //console.log(webSocketMessage.data)
            const messageBody = JSON.parse(webSocketMessage.data);
            updateState(messageBody.house)
        };

        ws.onopen = (event) => {
            console.log("Openned")
            ws.send("getState");

        }
        ws.onclose = (event) => {
            console.log("Closed")
            setTimeout(initWebSocket, 2000);
        }
    }


</script>
</body>
</html>
)rawliteral";