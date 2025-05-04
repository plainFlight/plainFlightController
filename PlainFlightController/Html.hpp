/* 
* Copyright (c) 2025 P.Cook (alias 'plainFlight')
*
* This file is part of the PlainFlightController distribution (https://github.com/plainFlight/plainFlightController).
* 
* This program is free software: you can redistribute it and/or modify  
* it under the terms of the GNU General Public License as published by  
* the Free Software Foundation, version 3.
*
* This program is distributed in the hope that it will be useful, but 
* WITHOUT ANY WARRANTY; without even the implied warranty of 
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License 
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/**
* @file   Html.hpp
* @brief  This class contains the HTML code that is sent over WiFi to act as the configurator interface.
*/

#pragma once

class Html
{
  protected:
     static constexpr char INDEX_HTML[] PROGMEM = R"rawliteral(
      <!DOCTYPE html>
      <html>
      <head>
        <meta name="keywords" content="HTML,CSS,JavaScript">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
      <head>
      <body>
      <style>
      h1 {
        border: 4px solid powderblue;
        background-color: gray;
        color: white;
        padding: 20px;
        font-family: verdana;
        font-size: 300%;
        text-align: center;
      }
      h2 {
        border: 4px solid powderblue;
        background-color: gray;
        color: white;
        padding: 20px;
        font-family: verdana;
        font-size: 100%;
        text-align: center;
      }
      p {
        color: red;
        text-align: center;
        font-weight:bold;
        }
      p1 {
        color: black;
        text-align: center;
        }
      input 
        {
        height: 100%;
        width: 20%;
        box-sizing: border-box;  
        text-align: center;
        font-size: 16px;
        }
      form {
        width: 100%;
        text-align: center;
      }
      legend {
        background-color: gray;
        color: white;
        padding: 5p 10px;
        border: 3px solid powderblue;
      }
      p.round {
        border: 2px solid blue;
        border-radius: 12px;
        padding: 5px;
      }
      fieldset{
        background-color: ghostwhite;
        border-radius: 8px;
      }

      </style>
      <h1>PlainFlightController %s<br>Configurator</h1><br><br>
      <p>For Safety Remove Propellers.</p>
        
      <fieldset> 
        <legend><b>Pitch Gains:</b></legend>
        <form action="/PITCH" method="get">
          P: <input type="number" name="P" value="%d" min="0" max="200"><br><br>
          I: <input type="number" name="I" value="%d" min="0" max="500"><br><br>  
          D: <input type="number" name="D" value="%d" min="0" max="2000"><br><br>   
          F: <input type="number" name="F" value="%d" min="0" max="50"><br><br>
        <input type="submit" value="Save"><br><br>
      </form>
      </fieldset>
      <br>

      <fieldset> 
        <legend><b>Roll Gains:</b></legend>
        <form action="/ROLL" method="get">
          P: <input type="number" name="P" value="%d" min="0" max="200"><br><br>
          I: <input type="number" name="I" value="%d" min="0" max="500"><br><br>  
          D: <input type="number" name="D" value="%d" min="0" max="2000"><br><br>   
          F: <input type="number" name="F" value="%d" min="0" max="50"><br><br>
        <input type="submit" value="Save"><br><br>
      </form>
      </fieldset>
      <br>
      
      <fieldset>
      <legend><b>Yaw Gains:</b></legend>
      <form action="/YAW" method="get">
        P: <input type="number" name="P" value="%d" min="0" max="200"><br><br>
        I: <input type="number" name="I" value="%d" min="0" max="500"><br><br>  
        D: <input type="number" name="D" value="%d" min="0" max="2000"><br><br>   
        F: <input type="number" name="F" value="%d" min="0" max="50"><br><br>
        <input type="submit" value="Save"><br><br>
      </form>
      </fieldset>
      <br>
    
      <fieldset>
      <legend><b>Rates:</b></legend>
      <form action="/RATES" method="get">
        Pitch (deg/s): <input type="number" name="pitch" value="%d" min="0" max="%d"><br><br>
        roll (deg/s): <input type="number" name="roll" value="%d" min="0" max="%d"><br><br>
        Yaw (deg/s): <input type="number" name="yaw" value="%d" min="0" max="%d"><br><br>   
        <input type="submit" value="Save"><br><br>
      </form>
      </fieldset>
      <br>      

      <fieldset>
      <legend><b>Max Self Levelled Angles:</b></legend> 
      <form action="/ANGLE" method="get">
        Pitch (degrees): <input type="number" name="pitch" value="%d" "min="0" max="90"><br><br>
        Roll (degrees): <input type="number" name="roll" value="%d" min="0" max="90"><br><br>   
        <input type="submit" value="Save"><br><br>
      </form>
      </fieldset>
      <br>

      <fieldset>
      <legend><b>Levelled Trims:</b></legend> 
      <form action="/LEVEL_TRIMS" method="get">
        <p1>Model current pitch: %.1f, roll: %.1f, yaw: %.1f ...<a href=\"javascript:window.location.reload();\">(CLICK TO REFRESH)</a></p1><br><br>
        Pitch (degrees): <input type="number" name="pitch" value="%.1f" min="-25.0" max="25.0" step="0.1"><br><br>  
        Roll (degrees): <input type="number" name="roll" value="%.1f" min="-25.0" max="25.0" step="0.1"><br><br>  
        Yaw (degrees): <input type="number" name="yaw" value="%.1f" min="-25.0" max="25.0" step="0.1"><br><br>
        <input type="submit" value="Save"><br><br>
      </form>
      </fieldset>
      <br>

      <fieldset>
      <legend><b>Servo Trims:</b></legend> 
      <form action="/SERVO_TRIMS" method="get">
        Servo 1: <input type="number" name="Servo1" value="%d" min="-200" max="200"><br><br>
        Servo 2: <input type="number" name="Servo2" value="%d" min="-200" max="200"><br><br>
        Servo 3: <input type="number" name="Servo3" value="%d" min="-200" max="200"><br><br>
        Servo 4: <input type="number" name="Servo4" value="%d" min="-200" max="200"><br><br>
        <input type="submit" value="Save"><br><br>
      </form>
      </fieldset>
      <br>

      <fieldset>
      <legend><b>Voltage Trim:</b></legend> 
      <form action="/VOLT_TRIM" method="get">
        <p1>Calculated: %.2f Volts ...<a href=\"javascript:window.location.reload();\">(CLICK TO REFRESH)</a></p1><br><br>
        Voltage scaler: <input type="number" name="volts" value="%.5f" min="0.0" max="0.05" step="0.00001"><br><br>  
        <input type="submit" value="Save"><br><br>
      </form>
      </fieldset>
      <br>

      <h2>Copyright 2025 P.Cook (alias 'plainFlight')<br><br>https://github.com/plainFlight/plainFlightController is licensed under the
      GNU General Public License v3.0<br><br>USE AT YOUR OWN RISK & LIABILITY</h2>
      </body>
      </html>)rawliteral";
};