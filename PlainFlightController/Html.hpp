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

/*
 * NOTE: The HTML, CSS and JS code in this file is used as a C printf() format string template.
 * Any literal % signs in the CSS must be escaped as %% to prevent them from being interpreted
 * as format specifiers.
 * Example: 50% must be written as 50%%.
 */

#pragma once

class Html
{
  protected:
     static constexpr char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
    <title>PlainFlightController Configurator</title>
    <style>
      *, *::before, *::after {
        box-sizing: border-box;
        margin: 0;
        padding: 0;
      }

      body {
        min-height: 100vh;
        display: flex;
        flex-direction: column;
        align-items: center;
        justify-content: center;
        background-color: #f0f0f0;
        font-family: sans-serif;
        padding: 0;
      }

      h1 {
        border: 4px solid powderblue;
        background-color: gray;
        color: white;
        padding: 20px;
        width: 90vw;
        margin: 0 auto 2rem;
        /* Better sizing: constrains to container while maximizing */
        font-size: clamp(1.5rem, 8vw, 3.5rem);
        line-height: 1.2;
        word-wrap: break-word;
        overflow-wrap: break-word;
        font-family: verdana;
        text-align: center;
      }

      fieldset {
        position: relative;
        background-color: ghostwhite;
        border-radius: 8px;
        width: 90vw;
        margin: 10px auto;
        padding: 10px 10px 24px 10px;
        border: 1px solid #ccc;
      }

      fieldset.locked .group::after {
        content: '';
        position: absolute;
        inset: 0;
        background-color: rgba(0, 0, 0, 0.15);
        border-radius: 6px;
        pointer-events: none;
      }

      fieldset.locked .group {
        pointer-events: none;
      }

      legend {
        text-align: left;
        margin: 0;
        margin-bottom: 1em;
        background-color: gray;
        font-weight: bold;
        color: white;
        padding: 5px 10px;
        border: 3px solid powderblue;
      }

      button, input, a {
        touch-action: manipulation;
      }

      p.warning {
        color: red;
        font-weight: bold;
        margin-bottom: 1rem;
      }

      p.footer {
        border: 4px solid powderblue;
        background-color: gray;
        color: white;
        width: 90vw;
        margin: 2rem auto;
        padding: 20px;
        font-size: clamp(1.1rem, 4vw, 1.6rem);
        line-height: 1.8;
        text-align: center;
      }

      .page {
        width: 100%%;
        text-align: center;
        margin: 0 auto;
      }

      .group {
        position: relative;
        background-color: #e8e8e8;
        border-radius: 6px;
        padding: 0 16px 10px 16px;
        display: inline-block;
      }

      .gains-grid {
        display: flex;
        flex-direction: column;
        gap: 2rem;
        margin-top: 2rem;
      }

      .gain-row {
        display: flex;
        align-items: center;
        justify-content: center;
        gap: 1rem;
        padding: 1.6rem 0 0 0;
      }

      .gain-row label {
        font-weight: bold;
        color: #333;
        flex-shrink: 0;
      }

      .container {
        position: relative;
        display: flex;
        flex-direction: column;
        align-items: center;
      }

      .quantity {
        display: flex;
        align-items: center;
        justify-content: center;
        box-shadow: 0 5px 10px rgba(0, 0, 0, 0.1);
        border-radius: 5px;
        overflow: hidden;
        background-color: #fff;
      }

      .quantity button {
        width: 45px;
        height: 45px;
        border: none;
        background-color: #eee;
        color: #333;
        font-size: 20px;
        cursor: pointer;
        transition: background-color 0.2s;
      }

      .quantity button:hover {
        background-color: #ddd;
      }

      .quantity input {
        width: 60px;
        height: 45px;
        text-align: center;
        border: none;
        font-size: 18px;
        color: #333;
        outline: none;
      }

      input[type="number"] {
        -moz-appearance: textfield;
        appearance: textfield;
      }

      input::-webkit-outer-spin-button,
      input::-webkit-inner-spin-button {
        -webkit-appearance: none;
        margin: 0;
      }

      input[name="volts"] {
        width: 100px;
        min-width: 100px;
      }

      .input-box {
        font-weight: normal;
      }

      .input-box:focus {
        font-weight: bold;
      }

      .input-box.changed {
        font-weight: bold;
      }

      .input-box.invalid {
        box-shadow: inset 0 0 0 2px #ff4d4d !important;
        border-radius: 0;
      }

      .info {
        position: absolute;
        bottom: 100%%;
        left: 50%%;
        transform: translateX(-50%%) translateY(5px);
        white-space: nowrap;
        margin-bottom: 12px;
        visibility: hidden;
        opacity: 0;
        font-size: 1rem;
        color: #666;
        transition: opacity 0.25s ease, transform 0.25s ease;
      }

      .info.visible {
        visibility: visible;
        opacity: 1;
        transform: translateX(-50) translateY(0);
      }

      .save-btn {
        margin-top: 0;
        margin-bottom: 0;
        width: 150px;
        height: 45px;
        border: none;
        border-radius: 5px;
        font-size: 16px;
        font-weight: bold;
        color: #fff;
        background-color: #bbb;
        cursor: not-allowed;
        box-shadow: 0 5px 10px rgba(0, 0, 0, 0.1);
        transition: all 0.3s ease;
      }

      .save-btn.active {
        background-color: #007bff;
        cursor: pointer;
      }

      .save-btn.active:hover {
        background-color: #0056b3;
        box-shadow: 0 5px 15px rgba(0, 123, 255, 0.3);
      }

      .btn-row {
        display: flex;
        align-items: center;
        justify-content: center;
        gap: 10px;
        margin-top: 30px;
        margin-bottom: 12px;
      }

      .reset-btn {
        width: 45px;
        height: 45px;
        border: none;
        border-radius: 5px;
        font-size: 24px;
        color: #fff;
        background-color: #bbb;
        cursor: not-allowed;
        box-shadow: 0 5px 10px rgba(0, 0, 0, 0.1);
        transition: all 0.3s ease;
      }

      .reset-btn.active {
        background-color: #e67e00;
        cursor: pointer;
      }

      .reset-btn.active:hover {
        background-color: #cf6f00;
        box-shadow: 0 5px 15px rgba(230, 126, 0, 0.3);
      }
    </style>
  </head>
  <body>
    <div class="page">
      <h1>PlainFlight&#8203;Controller %s<br>Configurator</h1>
      <p class="warning">For Safety Remove Propellers.</p>

      <form action="/PITCH" method="get">
        <fieldset>
          <legend>Pitch Gains</legend>
          <div class="group">
            <div class="gains-grid">
              <div class="gain-row">
                <label>P</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease P">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="P" value="%d" min="0" max="200">
                    <button type="button" class="plus" aria-label="Increase P">&plus;</button>
                  </div>
                </div>
              </div>
              <div class="gain-row">
                <label>I</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease I">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="I" value="%d" min="0" max="500">
                    <button type="button" class="plus" aria-label="Increase I">&plus;</button>
                  </div>
                </div>
              </div>
              <div class="gain-row">
                <label>D</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease D">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="D" value="%d" min="0" max="2000">
                    <button type="button" class="plus" aria-label="Increase D">&plus;</button>
                  </div>
                </div>
              </div>
              <div class="gain-row">
                <label>F</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease F">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="F" value="%d" min="0" max="50">
                    <button type="button" class="plus" aria-label="Increase F">&plus;</button>
                  </div>
                </div>
              </div>
            </div>
            <div class="btn-row">
              <button type="button" class="reset-btn" disabled>&#x21BA;</button>
              <button type="submit" class="save-btn" disabled>Save</button>
            </div>
          </div>
        </fieldset>
      </form>

      <form action="/ROLL" method="get">
        <fieldset>
          <legend>Roll Gains</legend>
          <div class="group">
            <div class="gains-grid">
              <div class="gain-row">
                <label>P</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease P">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="P" value="%d" min="0" max="200">
                    <button type="button" class="plus" aria-label="Increase P">&plus;</button>
                  </div>
                </div>
              </div>
              <div class="gain-row">
                <label>I</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease I">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="I" value="%d" min="0" max="500">
                    <button type="button" class="plus" aria-label="Increase I">&plus;</button>
                  </div>
                </div>
              </div>
              <div class="gain-row">
                <label>D</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease D">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="D" value="%d" min="0" max="2000">
                    <button type="button" class="plus" aria-label="Increase D">&plus;</button>
                  </div>
                </div>
              </div>
              <div class="gain-row">
                <label>F</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease F">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="F" value="%d" min="0" max="50">
                    <button type="button" class="plus" aria-label="Increase F">&plus;</button>
                  </div>
                </div>
              </div>
            </div>
            <div class="btn-row">
              <button type="button" class="reset-btn" disabled>&#x21BA;</button>
              <button type="submit" class="save-btn" disabled>Save</button>
            </div>
          </div>
        </fieldset>
      </form>

      <form action="/YAW" method="get">
        <fieldset>
          <legend>Yaw Gains</legend>
          <div class="group">
            <div class="gains-grid">
              <div class="gain-row">
                <label>P</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease P">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="P" value="%d" min="0" max="200">
                    <button type="button" class="plus" aria-label="Increase P">&plus;</button>
                  </div>
                </div>
              </div>
              <div class="gain-row">
                <label>I</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease I">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="I" value="%d" min="0" max="500">
                    <button type="button" class="plus" aria-label="Increase I">&plus;</button>
                  </div>
                </div>
              </div>
              <div class="gain-row">
                <label>D</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease D">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="D" value="%d" min="0" max="2000">
                    <button type="button" class="plus" aria-label="Increase D">&plus;</button>
                  </div>
                </div>
              </div>
              <div class="gain-row">
                <label>F</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease F">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="F" value="%d" min="0" max="50">
                    <button type="button" class="plus" aria-label="Increase F">&plus;</button>
                  </div>
                </div>
              </div>
            </div>
            <div class="btn-row">
              <button type="button" class="reset-btn" disabled>&#x21BA;</button>
              <button type="submit" class="save-btn" disabled>Save</button>
            </div>
          </div>
        </fieldset>
      </form>

      <form action="/RATES" method="get">
        <fieldset>
          <legend>Rates</legend>
          <div class="group">
            <div class="gains-grid">
              <div class="gain-row">
                <label>Pitch (deg/s)</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease Pitch">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="pitch" value="%d" min="0" max="%d">
                    <button type="button" class="plus" aria-label="Increase Pitch">&plus;</button>
                  </div>
                </div>
              </div>
              <div class="gain-row">
                <label>Roll (deg/s)</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease Roll">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="roll" value="%d" min="0" max="%d">
                    <button type="button" class="plus" aria-label="Increase Roll">&plus;</button>
                  </div>
                </div>
              </div>
              <div class="gain-row">
                <label>Yaw (deg/s)</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease Yaw">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="yaw" value="%d" min="0" max="%d">
                    <button type="button" class="plus" aria-label="Increase Yaw">&plus;</button>
                  </div>
                </div>
              </div>
            </div>
            <div class="btn-row">
              <button type="button" class="reset-btn" disabled>&#x21BA;</button>
              <button type="submit" class="save-btn" disabled>Save</button>
            </div>
          </div>
        </fieldset>
      </form>

      <form action="/ANGLE" method="get">
        <fieldset>
          <legend>Max Self Levelled Angles</legend>
          <div class="group">
            <div class="gains-grid">
              <div class="gain-row">
                <label>Pitch (degrees)</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease Pitch">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="pitch" value="%d" min="0" max="90">
                    <button type="button" class="plus" aria-label="Increase Pitch">&plus;</button>
                  </div>
                </div>
              </div>
              <div class="gain-row">
                <label>Roll (degrees)</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease Roll">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="roll" value="%d" min="0" max="90">
                    <button type="button" class="plus" aria-label="Increase Roll">&plus;</button>
                  </div>
                </div>
              </div>
            </div>
            <div class="btn-row">
              <button type="button" class="reset-btn" disabled>&#x21BA;</button>
              <button type="submit" class="save-btn" disabled>Save</button>
            </div>
          </div>
        </fieldset>
      </form>

      <form action="/LEVEL_TRIMS" method="get">
        <fieldset>
          <legend>Levelled Trims</legend>
          <div class="group">
            <div class="gains-grid">
              <p class="p1">Model current:<br>pitch: %.1f, roll: %.1f, yaw: %.1f<br>
                <a href="javascript:window.location.reload();">(CLICK TO REFRESH)</a>
              </p>
              <div class="gain-row">
                <label>Pitch (degrees)</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease Pitch">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="pitch" value="%.1f" min="-25.0" max="25.0" step="0.1">
                    <button type="button" class="plus" aria-label="Increase Pitch">&plus;</button>
                  </div>
                </div>
              </div>
              <div class="gain-row">
                <label>Roll (degrees)</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease Roll">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="roll" value="%.1f" min="-25.0" max="25.0" step="0.1">
                    <button type="button" class="plus" aria-label="Increase Roll">&plus;</button>
                  </div>
                </div>
              </div>
              <div class="gain-row">
                <label>Yaw (degrees)</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease Yaw">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="yaw" value="%.1f" min="-25.0" max="25.0" step="0.1">
                    <button type="button" class="plus" aria-label="Increase Yaw">&plus;</button>
                  </div>
                </div>
              </div>
            </div>
            <div class="btn-row">
              <button type="button" class="reset-btn" disabled>&#x21BA;</button>
              <button type="submit" class="save-btn" disabled>Save</button>
            </div>
          </div>
        </fieldset>
      </form>

      <form action="/SERVO_TRIMS" method="get">
        <fieldset>
          <legend>Servo Trims</legend>
          <div class="group">
            <div class="gains-grid">
              <div class="gain-row">
                <label>Servo 1</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease Servo 1">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="Servo1" value="%d" min="-200" max="200">
                    <button type="button" class="plus" aria-label="Increase Servo 1">&plus;</button>
                  </div>
                </div>
              </div>
              <div class="gain-row">
                <label>Servo 2</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease Servo 2">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="Servo2" value="%d" min="-200" max="200">
                    <button type="button" class="plus" aria-label="Increase Servo 2">&plus;</button>
                  </div>
                </div>
              </div>
              <div class="gain-row">
                <label>Servo 3</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease Servo 3">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="Servo3" value="%d" min="-200" max="200">
                    <button type="button" class="plus" aria-label="Increase Servo 3">&plus;</button>
                  </div>
                </div>
              </div>
              <div class="gain-row">
                <label>Servo 4</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease Servo 4">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="Servo4" value="%d" min="-200" max="200">
                    <button type="button" class="plus" aria-label="Increase Servo 4">&plus;</button>
                  </div>
                </div>
              </div>
            </div>
            <div class="btn-row">
              <button type="button" class="reset-btn" disabled>&#x21BA;</button>
              <button type="submit" class="save-btn" disabled>Save</button>
            </div>
          </div>
        </fieldset>
      </form>

      <form action="/VOLT_TRIM" method="get">
        <fieldset>
          <legend>Voltage Trim</legend>
          <div class="group">
            <div class="gains-grid">
              <p class="p1">Calculated: %.2f Volts<br>
                <a href="javascript:window.location.reload();">(CLICK TO REFRESH)</a>
              </p>
              <div class="gain-row">
                <label>Scaler</label>
                <div class="container">
                  <div class="info"></div>
                  <div class="quantity">
                    <button type="button" class="minus" aria-label="Decrease Voltage Scaler">&minus;</button>
                    <input class="input-box" type="text" inputmode="decimal" lang="en" name="volts" value="%.5f" min="0.0" max="0.05" step="0.00001">
                    <button type="button" class="plus" aria-label="Increase Voltage Scaler">&plus;</button>
                  </div>
                </div>
              </div>
            </div>
            <div class="btn-row">
              <button type="button" class="reset-btn" disabled>&#x21BA;</button>
              <button type="submit" class="save-btn" disabled>Save</button>
            </div>
          </div>
        </fieldset>
      </form>

      <p class="footer">Copyright 2025 P.Cook (alias 'plainFlight')<br><br>
        https://github.com&#8203;/&#8203;plainFlight&#8203;/&#8203;plainFlightController is licensed under the GNU General Public License v3.0<br><br>
        USE AT YOUR OWN RISK & LIABILITY</p>
    </div>
    <script>
      document.querySelectorAll("fieldset").forEach(initFieldset);

      function initFieldset(fieldset) {
        const saveBtn = fieldset.querySelector(".save-btn");
        const resetBtn = fieldset.querySelector(".reset-btn");

        resetBtn.addEventListener("click", () => {
          if (resetBtn.disabled) return;
          fieldset.querySelectorAll(".input-box").forEach(i => {
            i.value = i.dataset.initial;
            i.classList.remove("changed", "invalid");
          });
          fieldset.querySelectorAll(".info").forEach(info => {
            info.classList.remove("visible");
          });
          saveBtn.disabled = true;
          saveBtn.classList.remove("active");
          resetBtn.disabled = true;
          resetBtn.classList.remove("active");
          unlockAllFieldsets();
        });

        fieldset.closest("form").addEventListener("submit", (e) => {
          if (saveBtn.disabled) {
            e.preventDefault();
            return;
          }
          fieldset.querySelectorAll(".input-box").forEach((i) => {
            i.value = i.value.replace(",", "."); // Ensure dot as decimal separator
          });
        });

        function lockOtherFieldsets(activeFieldset) {
          document.querySelectorAll("fieldset").forEach((el) => {
            if (el !== activeFieldset) {
              el.classList.add("locked");
              el.querySelectorAll(".input-box, .minus, .plus, .save-btn").forEach(
                (btn) => {
                  btn.disabled = true;
                }
              );
            }
          });
        }

        function unlockAllFieldsets() {
          document.querySelectorAll("fieldset").forEach((el) => {
            el.classList.remove("locked");
            el.querySelectorAll(".input-box, .minus, .plus").forEach((btn) => {
              btn.disabled = false;
            });
          });
        }

        fieldset.querySelectorAll(".input-box").forEach((input) => {
          const minus = input.closest(".quantity").querySelector(".minus");
          const plus = input.closest(".quantity").querySelector(".plus");
          const infoBox = input.closest(".container").querySelector(".info");

          const initial = parseFloat(input.value);
          const min = parseValue(input.min);
          const max = parseValue(input.max);
          const step = parseValue(input.step) || 1;
          const decimals = step.toString().includes(".")
            ? step.toString().split(".")[1].length
            : 0;

          function updateInfoBox() {
            infoBox.textContent = `${initial} [${min} / ${max}]`;
            infoBox.classList.toggle(
              "visible",
              document.activeElement === input || parseFloat(input.value) !== initial
            );
          }

          function validate() {
            input.value = input.value.replace(/(?!^)-/g, "");
            const val = parseFloat(input.value.replace(',', '.'));
            const isInvalid = isNaN(val) || val < min || val > max || String(val) !== input.value.replace(',', '.');
            input.classList.toggle("invalid", isInvalid);
            input.classList.toggle("changed", val !== parseFloat(input.dataset.initial));
            updateSaveBtn();
          }

          function updateSaveBtn() {
            const allInputs = [...fieldset.querySelectorAll(".input-box")];
            const anyChanged = allInputs.some(i => parseFloat(i.value.replace(',', '.')) !== parseFloat(i.dataset.initial));
            const anyInvalid = allInputs.some(i => i.classList.contains("invalid"));
            const enable = anyChanged && !anyInvalid;
            saveBtn.disabled = !enable;
            saveBtn.classList.toggle("active", enable);
            resetBtn.disabled = !anyChanged;
            resetBtn.classList.toggle("active", anyChanged);

            if (anyChanged) {
              lockOtherFieldsets(fieldset);
            } else {
              unlockAllFieldsets();
            }
          }

          minus.addEventListener("click", () => {
            const val = parseValue(input.value);
            if (!isNaN(val) && val > min) {
              input.value = parseValue((val - step).toFixed(decimals));
              validate();
              updateInfoBox();
            }
          });

          plus.addEventListener("click", () => {
            const val = parseValue(input.value);
            if (!isNaN(val) && val < max) {
              input.value = parseValue((val + step).toFixed(decimals));
              validate();
              updateInfoBox();
            }
          });

          input.addEventListener("input", () => {
            input.value = input.value.replace(/[^0-9\-\.]/g, "");
            validate();
            updateInfoBox();
          });
          input.addEventListener("focus", updateInfoBox);
          input.addEventListener("blur", () => {
            if (input.value === "") {
              input.value = initial;
              validate();
            }
            updateInfoBox();
          });
          input.addEventListener("keydown", (e) => {
            if (e.key === "Enter") {
              e.preventDefault();
              validate();
            }
          });

          input.dataset.initial = initial;
        });
      }

      function parseValue(str) {
        return parseFloat(str.replace(",", "."));
      }

      // Call this whenever saveBtn state changes in any fieldset
      function isAnyFieldsetDirty() {
        return [...document.querySelectorAll(".save-btn")].some((btn) =>
          btn.classList.contains("active")
        );
      }

      document
        .querySelectorAll('a[href="javascript:window.location.reload();"]')
        .forEach((link) => {
          link.addEventListener("click", (e) => {
            if (isAnyFieldsetDirty()) {
              if (!confirm("You have unsaved changes.\nRefresh anyway?")) {
                e.preventDefault();
              }
            }
          });
        });

      let lastTap = 0;

      document.addEventListener(
        "touchend",
        (e) => {
          const now = Date.now();
          if (now - lastTap < 300) {
            e.preventDefault(); // block zoom on double-tap
          }
          lastTap = now;
        },
        { passive: false }
      );
    </script>
  </body>
</html>)rawliteral";
};