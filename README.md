# Clickfy – BLE Multimedia Remote

## Description

Clickfy is a compact ESP32-based Bluetooth Low Energy multimedia remote designed as a portable keychain controller. The device allows users to control basic media functions such as play/pause, next track and previous track wirelessly.

The main goal of the project was to create a small, battery-powered device with low power consumption and fast response time.

## Features

* Bluetooth Low Energy (BLE) multimedia control
* Play / Pause
* Next track
* Previous track
* Battery voltage monitoring
* Low-power operation
* Compact custom PCB
* Portable keychain-sized design

## Hardware

* ESP32-C3
* Li-Po battery
* Push buttons
* Status LED
* Custom PCB design

## Software

* C/C++
* BLE HID profile
* Battery measurement using ADC
* Power management and sleep modes

## Challenges

Some of the main development challenges included:

* minimizing power consumption,
* reducing wake-up and reconnection time after sleep,
* implementing reliable BLE communication,
* designing a compact PCB layout.

## Current status

Working prototype built and tested.

Future improvements:

* further firmware power optimization,
* faster BLE wake-up behavior,
* additional multimedia functions.
