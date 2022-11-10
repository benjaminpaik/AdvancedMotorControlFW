# AdvancedMotorControl

Embedded motor control software demo for the STM32G4.

## Overview

The purpose of this project is to provide embedded software 
infrastructure for the STM32G4 for motor control applications. This
was developed using a custom PCB that provides two general-purpose
analog inputs, a boot mode switch, hall sensor feedback for 
commutation, quadrature encoder inputs, a temperature sensor, phase 
A/B current sensors, Bluetooth/USB/RS-485 communication options, and 
two status LEDs. This software demonstrates the implementation of each 
of these features to produce a state space controller for an inverted
pendulum.
