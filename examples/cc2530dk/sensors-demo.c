/*
 * Copyright (c) 2010, Loughborough University - Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

/**
 * \file
 *         Example to demonstrate-test cc2530 sensor functionality
 *
 *         B1 turns LED_GREEN on and off.
 *
 *         The node takes readings from the various sensors every x seconds and
 *         prints out the results.
 *
 *         We use floats here to translate the AD conversion results to
 *         meaningful values. However, our printf does not have %f support so
 *         we use an ugly hack to print out the value by extracting the integral
 *         part and then the fractional part. Don't try this at home.
 *
 *         Temperature:
 *           Math is correct, the sensor needs calibration per device.
 *           I currently use default values for the math which may result in
 *           very incorrect values in degrees C.
 *           See TI Design Note DN102 about the offset calibration.
 *
 *         Supply Voltage (VDD):
 *           For VDD, math is correct, conversion is correct.
 *           See DN101 for details.
 *
 *         Make sure you enable/disable things in contiki-conf.h
 *
 * \author
 *         George Oikonomou - <oikonomou@users.sourceforge.net>
 */

#include "contiki.h"
#include "contiki-conf.h"
#include "dev/leds.h"

#include "dev/button-sensor.h"
#include "dev/adc-sensor.h"
#include "dev/bmp280-sensor.h"

#define DEBUG 1

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else /* DEBUG */
/* We overwrite (read as annihilate) all output functions here */
#define PRINTF(...)
#endif /* DEBUG */
/*---------------------------------------------------------------------------*/
PROCESS(sensors_test_process, "Sensor Test Process");
#if BUTTON_SENSOR_ON
PROCESS(buttons_test_process, "Button Test Process");
AUTOSTART_PROCESSES(&sensors_test_process, &buttons_test_process);
#else
AUTOSTART_PROCESSES(&sensors_test_process);
#endif
/*---------------------------------------------------------------------------*/
#if BUTTON_SENSOR_ON
PROCESS_THREAD(buttons_test_process, ev, data)
{
	struct sensors_sensor *sensor;

	PROCESS_BEGIN();

	while(1) {

		PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event);

		/* If we woke up after a sensor event, inform what happened */
		sensor = (struct sensors_sensor *)data;
		if(sensor == &button_sensor) {
			PRINTF("Button Press\n");
			leds_toggle(LEDS_GREEN);
		}
	}

	PROCESS_END();
}
#endif
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sensors_test_process, ev, data)
{
	static struct etimer et;

	/* Sensor Values */
	static int rv;
	static int pressurePa_MSB, pressurePa_LSB;
	static struct sensors_sensor *sensor;
	static struct sensors_sensor *sensor2;
	static float sane = 0;
	static int dec;
	static float frac;
	char buf;
	//struct bmp280_t bmp280Sensor;l
	PROCESS_BEGIN();

	PRINTF("========================\n");
	PRINTF("Starting Sensor Example.\n");
	PRINTF("========================\n");
	//I2C_routine();
	//  HalI2CInit();
	//  // Configura o pino P1.7 e seta o valor para 0, de modo que o endereço
	//  // do BMP280 seja 0x76
	//  P1SEL &= ~SDO_MASK;
	//  P1DIR |= SDO_MASK;
	//  P1_7 |= SDO_MASK;
	/* Set an etimer. We take sensor readings when it expires and reset it. */
	etimer_set(&et, CLOCK_SECOND * 2);

	while(1)
	{
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		/*
		 * Request some ADC conversions
		 * Return value -1 means sensor not available or turned off in conf
		 */
		sensor = sensors_find(ADC_SENSOR);
		if(sensor)
		{
			PRINTF("------------------\n");
			leds_on(LEDS_RED);
			/*
			 * Temperature:
			 * Using 1.25V ref. voltage (1250mV).
			 * Typical AD Output at 25°C: 1480
			 * Typical Co-efficient     : 4.5 mV/°C
			 *
			 * Thus, at 12bit decimation (and ignoring the VDD co-efficient as well
			 * as offsets due to lack of calibration):
			 *
			 *          AD - 1480
			 * T = 25 + ---------
			 *              4.5
			 */
			rv = sensor->value(ADC_SENSOR_TYPE_TEMP);
			if(rv != -1) {
				sane = 25 + ((rv - 1480) / 4.5);
				dec = sane;
				frac = sane - dec;
				PRINTF("  Temp=%d.%02u C (%d)\n", dec, (unsigned int)(frac*100), rv);
				leds_toggle(LEDS_BLUE);
				buf = leds_get();
				PRINTF("LED_STATE: %s %s %s %s\n",(buf&LEDS_GREEN)?" (G) ":"  G  ",(buf&LEDS_RED)?" (R) ":"  R  ",(buf&LEDS_BLUE)?" (B) ":"  B  ",(buf&LEDS_YELLOW)?" (Y) ":"  Y  ");
			}
			/*
			 * Power Supply Voltage.
			 * Using 1.25V ref. voltage.
			 * AD Conversion on VDD/3
			 *
			 * Thus, at 12bit resolution:
			 *
			 *          ADC x 1.25 x 3
			 * Supply = -------------- V
			 *               2047
			 */
			rv = sensor->value(ADC_SENSOR_TYPE_VDD);
			if(rv != -1) {
				//sane = rv * 3.75 / 2047;
				sane = ((3*(1.0512))*rv)/(2047-rv*0.12);
				dec = sane;
				frac = sane - dec;



				PRINTF("Supply=%d.%02u V (%d)\n", dec, (unsigned int)(frac*100), rv);
				/* Store rv temporarily in dec so we can use it for the battery */
				dec = rv;
			}

			rv = sensor->value(ADC_SENSOR_TYPE_AIN6);
			if(rv != -1) {
				PRINTF("AIN6 Reading=%d\n", rv);
			}

			/*
			 * Battery Voltage - ToDo
			 *   rv = sensor->value(ADC_SENSOR_TYPE_BATTERY);
			 */

			//bmp280Sensor.BMP280_BUS_READ_FUNC(bmp280Sensor.dev_addr,BMP280_CHIP_ID_REG, &bmp280ID,BMP280_GEN_READ_WRITE_DATA_LENGTH);

			//hali2cSend(bmp280ID, 2, SEND_START, NOSEND_STOP);
			//hali2cSend(bmp280ID+2, 1, SEND_START, NOSEND_STOP);
			//bmp280ID[2] = hali2cReceiveByte();
			//hali2cStop();//hali2cWrite(SMB_ACK);

			//      HalI2CReceive(0x77,0xD0,bmp280ID, 1);
			//      PRINTF("BMP280 ID=%2X\n", bmp280ID[0]);
			//      HalI2CSend(0x77,bmp280TempPressConfig+1,2);
			//
			//      HalI2CReceive(0x77,bmp280TempPressConfig[1],bmp280TempPressReadConfig, 1);
			//      PRINTF("BMP280 Config=%2X\n", bmp280TempPressReadConfig[0]);
			//      HalI2CReceive(bmp280TempPress[0], bmp280TempPress, 6);
			//      //PRINTF("BMP280 Pressure=%u\n", (uint32_t)(bmp280TempPress[3]<<12) | (uint32_t)(bmp280TempPress[4]<<4) | (uint32_t)(bmp280TempPress[5] >> 4));
			//      //PRINTF("BMP280 Temperature=%u\n", (uint32_t)(bmp280TempPress[6]<<12) | (uint32_t)(bmp280TempPress[7]<<4) | (uint32_t)(bmp280TempPress[8] >> 4));
			//
			//      PRINTF("BMP280 Pressure=%2X %2X %2X\n", bmp280TempPress[3],bmp280TempPress[4],bmp280TempPress[5]);
			//      PRINTF("BMP280 Temperature=%2X %2X %2X\n", bmp280TempPress[6],bmp280TempPress[7],bmp280TempPress[8]);

			leds_off(LEDS_RED);
		}
		sensor2 = sensors_find(BMP_SENSOR);
		if(sensor2)
		{
			rv = sensor2->value(BMP_SENSOR_TYPE_TEMP);
			if(rv != -1)
			{
				PRINTF("BMP Temp=%d.%02u C\n", rv/100, (unsigned int)(rv%100));
			}
			rv = sensor2->value(BMP_SENSOR_TYPE_PRESS_hPa);
			if(rv != -1)
			{
				PRINTF("BMP Press=%d hPa\n", rv);
				pressurePa_MSB = sensor2->value(BMP_SENSOR_TYPE_LAST_READ_PRESS_Pa_MSB);
				pressurePa_LSB = sensor2->value(BMP_SENSOR_TYPE_LAST_READ_PRESS_Pa_LSB);
				PRINTF("BMP Press=%ld Pa\n", ((int32_t)(pressurePa_MSB))<<16 | (int32_t)pressurePa_LSB);
			}
		}
		etimer_reset(&et);
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
