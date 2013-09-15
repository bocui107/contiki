/*
 * Copyright (c) 2008, Technical University of Munich
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
 *
 * @(#)$$
 */

/**
 * \addtogroup usbstick
 *
 */

#ifndef __CONTIKI_RAVEN_H__
#define __CONTIKI_RAVEN_H__

#include "contiki.h"
#include "contiki-net.h"
#include "contiki-lib.h"

/*
 * LED's for Raven USB
 * LED1: PD7 ---> Led0
 * LED2: PD5 ---> Led1
 * LED3: PE7 ---> Led2
 * LED4: PE6 ---> Led3
 */

/*
 * Each port pin consists of three register bits: DDxn, PORTxn, and PINxn.
 *
 * The DDxn bit in the DDRx Register selects the direction of this pin.
 * If DDxn is written logic one, Pxn is configured as an output pin.
 * If DDxn is written logic zero, Pxn is configured as an input pin.
 *
 * If PORTxn is written logic one when the pin is configured as an input pin,
 * the pull-up resistor is activated. To switch the pull-up resistor off,
 * PORTxn has to be written logic zero or the pin has to be configured as an
 * output pin.
 *
 * If PORTxn is written logic one when the pin is configured as an output pin,
 * the port pin is driven high (one). If PORTxn is written logic zero when the
 * pin is configured as an output pin, the port pin is driven low (zero).
 *
 * However, writing a logic one to a bit in the PINx Register, will result
 * in a toggle in the correspond-ing bit in the Data Register.
 *
 * In addition, the Pull-up Disable – PUD bit in MCUCR disables the
 * pull-up function for all pins in all ports when set.
 */
#define Leds_init()                 (DDRD  |=  0xA0, DDRE  |=  0xC0)
/*
 * Note: The Led0 port set 1 to turu on the led1
 * Because the LED1 connect to a triode(三极管, A NPN type), After
 * PORTD(LED1) have some change, the LED1 will be turn on
 */
#define Led0_on()                   (PORTD |=  0x80)
#define Led1_on()                   (PORTD &= ~0x20)
#define Led2_on()                   (PORTE &= ~0x80)
#define Led3_on()                   (PORTE &= ~0x40)
#define Led0_off()                  (PORTD &= ~0x80)
#define Led1_off()                  (PORTD |=  0x20)
#define Led2_off()                  (PORTE |=  0x80)
#define Led3_off()                  (PORTE |=  0x40)
#define Led0_toggle()               (PIND |= 0x80)
#define Led1_toggle()               (PIND |= 0x20)
#define Led2_toggle()               (PINE |= 0x80)
#define Led3_toggle()               (PINE |= 0x40)
#define Leds_on()                   (Led0_on(),Led1_on(),Led2_on(),Led3_on())
#define Leds_off()                  (Led0_off(),Led1_off(),Led2_off(),Led3_off())

void init_lowlevel(void);
void init_net(void);

#endif /* #ifndef __CONTIKI_RAVEN_H__ */
