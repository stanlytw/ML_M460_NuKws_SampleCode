# M460 Series

To experience the powerful features of M460 in few minutes, please refer to NuMaker-PFM-M460 Board Quick Start Guide. You can select the sample code of your interest to download and execute on the M460 board. You can open the project files to build them with Keil® MDK, IAR or Eclipse, and then download and trace them on the M460 board to see how it works.

## .\Document\


- CMSIS.html<br>
	Introduction of CMSIS version 5.0. CMSIS components included CMSIS-CORE, CMSIS-Driver, CMSIS-DSP, etc.

- NuMicro M460 Series CMSIS BSP Revision History.pdf<br>
	The revision history of M460 Series BSP.

- NuMicro M460 Series Driver Reference Guide.chm<br>
	The usage of drivers in M460 Series BSP.

## .\Library\


- CMSIS<br>
	Cortex® Microcontroller Software Interface Standard (CMSIS) V5.0 definitions by ARM® Corp.

- CryptoAccellerator<br>
	Library for mbed TLS crypto.

- Device<br>
	CMSIS compliant device header file.

- NuMaker<br>
	Specific libraries for M460 NuMaker board.

- SmartcardLib<br>
	Library for accessing a smartcard.

- StdDriver<br>
	All peripheral driver header and source files.

- UsbHostLib<br>
	USB host library source code.

## .\Sample Code\

- tflu_kws_arm or tflu_kws_arm_mc<br>
	KWS offline. Run DNN/DS-CNN for testing different saved PCM data in raw folder.
	
- tflu_kws_arm_rt or tflu_kws_arm_rt_mc<br>
	KWS realtime. Run DNN/DS-CNN for inference realtime.
	
- tflu_sin<br>
	An easy example to use tflite-micro.	

## .\ThirdParty\

- FatFs<br>
	An open source FAT/exFAT filesystem library.

- FreeRTOS<br>
	FreeRTOS porting for M460.

- libjpeg<br>
	A software implements JPEG baseline, extended-sequential, and progressive compression processes maintained and published by the Independent JPEG Group (IJG).

- LibMAD<br>
	A MPEG audio decoder library that currently supports MPEG-1 and the MPEG-2 extension to lower sampling frequencies, as well as the de facto MPEG 2.5 format.

- lwIP<br>
	A widely used open source TCP/IP stack designed for embedded systems.

- mbedtls-2.13.0<br>
	A portable, easy to use, readable and flexible SSL library.

- mbedtls-3.1.0<br>
	mbed TLS offers an SSL library with an intuitive API and readable source code, so you can actually understand what the code does.

- paho.mqtt.embedded-c<br>
	Eclipse Paho MQTT C/C++ client for Embedded platforms.

- shine<br>
	A blazing fast MP3 encoding library implemented in fixed-point arithmetic.

- tflite_micro<br>
	A tensorflow lite for micro library.
# Licesne

**SPDX-License-Identifier: Apache-2.0**

Copyright in some of the content available in this BSP belongs to third parties.
Third parties license is specified in a file header or license file.
M460 BSP files are provided under the Apache-2.0 license.

