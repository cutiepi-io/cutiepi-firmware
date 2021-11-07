/*
 *   ******************************************************************************
 *  Copyright (c) 2019-2020 Lijun.Cui
 *  All rights reserved.
 *
 *  Created on: 05/19/2019
 *  Author: Lijun.Cui
 *  mail: cljun08@163.com
 *   ******************************************************************************
 */

#ifndef ADC_H_
#define ADC_H_

#define USB_CHARGE_THRESHOLD_VAL 3000 /// mV

void ADC_update();

uint16_t get_battery_voltage();
uint16_t get_usb_voltage();

#endif /* ADC_H_ */
