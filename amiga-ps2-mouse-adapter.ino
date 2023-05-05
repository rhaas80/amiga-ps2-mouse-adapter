/*
 * Convert PS/2 mouse movements to Amiga mouse movements.
 * 
 * Copyright (c) 2020 Andreas Signer <asigner@gmail.com>
 * Copyright (c) 2023 Roland Haas <rhaas80@gmail.com>
 *
 * This file is part of amiga-ps2-mouse-adapter.
 * 
 * amiga-ps2-mouse-adapter is free software: you can redistribute it 
 * and/or modify it under the terms of the GNU General Public License 
 * as published by the Free Software Foundation, either version 3 of 
 * the License, or (at your option) any later version.
 * 
 * amiga-ps2-mouse-adapter is distributed in the hope that it will be 
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "config.h"
#include "ps2.h"

#include <avr/pgmspace.h>

#define LED 13

// Input pins
#define P_PS2_CLK  3
#define P_PS2_DATA 2

/*
 * PS/2 pinout female (computer port) connector from the front
 *    _   _
 *   / |_| \
 *  / 6   5 \
 * < 4     3 >
 *  \ 2   1 /
 *   - - - -
 *
 *   1 DATA
 *   2 N/C
 *   3 GND
 *   4 +5V
 *   5 CLK
 *   6 N/C
 */

// Output pins              // Cable color to D-sub 9
#define P_AMIGA_V_PULSE  12 // Purple
#define P_AMIGA_H_PULSE  11 // Brown
#define P_AMIGA_VQ_PULSE 10 // Blue
#define P_AMIGA_HQ_PULSE  9 // White
#define P_AMIGA_LMB       8 // Yellow
#define P_AMIGA_RMB       7 // Green
#define P_AMIGA_MMB       6 // Orange

/*
 * Amiga mouse-port, looking at the amiga:
 *
 * 9 PIN D-SUB MALE
 *
 *   _________________________
 *   \                       /
 *    \  1   2   3   4   5  /
 *     \                   /
 *      \  6   7   8   9  /
 *       \_______________/
 *
 * 1  V-pulse
 * 2  H-pulse
 * 3  VQ-pulse
 * 4  HQ-pulse
 * 5  BUTTON 3 (Middle)
 * 6  BUTTON 1 (Lefl)
 * 7  +5V
 * 8  GND
 * 9  BUTTON 2 (Right)
 */

static inline int sgn(int val) {
  return (val < 0) ? -1 : ((val > 0) ? 1 : 0);
}

/*
 * From "Amiga Intern", page 258:
 * 
 * Right/Down: H/V:      ______        _____
 *                      |      |      |
 *                      |      |      |
 *                     _|      |______|
 * 
 *             HQ/VQ:        ______        _
 *                          |      |      |
 *                          |      |      |
 *                     _____|      |______|
 *         
 *         
 * Left/Up:    H/V:          ______        _
 *                          |      |      |
 *                          |      |      |
 *                     _____|      |______|
 * 
 *             HQ/VQ:    ______        _____
 *                      |      |      |
 *                      |      |      |
 *                     _|      |______|
 *         
 */   

// pulse pattern for moving right/down.
const byte p[4] PROGMEM = { 1, 1, 0, 0 };
const byte pq[4] PROGMEM = { 0, 1, 1, 0 };

// position in pattern for x and y movement
byte posX = 0;
byte posY = 0;

void setupMouse() {
  ps2SendCommand(PS2_CMD_RESET);
  ps2Receive(); // read and ignore status response
  ps2Receive(); // read and ignore device ID

  // Amiga generates 200 impulses/inch, which is  
  // 0.78 impulses/mm, so let's go with a resolution
  // of 1 count per mm.
  // Add a factor of 4 for impulses vs state change
  // of quadrature code
  ps2SendCommand(PS2_CMD_SET_RESOLUTION);
  ps2SendCommand(PS2_RES_4_CNT_PER_MM);

  // Enable data then stop mouse
  ps2SendCommand(PS2_CMD_ENABLE_DATA_REPORTING);
  ps2Inhibit();
}

void blink() {
  for (int i = 0; i < 2; i++) {
    delay(200);
    digitalWrite(LED, HIGH);
    delay(200);
    digitalWrite(LED, LOW);    
  }
}

void setup() {
  ps2SetPins(P_PS2_CLK, P_PS2_DATA);

#if defined(DEBUG) || defined(DEBUG_PS2)
  Serial.begin(115200);
#endif  

  // Blink a little to let people know that we entered setup()
  pinMode(LED, OUTPUT);
  blink();

  // Set up pin modes
  pinMode(P_AMIGA_V_PULSE,  INPUT);
  pinMode(P_AMIGA_H_PULSE,  INPUT);
  pinMode(P_AMIGA_VQ_PULSE, INPUT);
  pinMode(P_AMIGA_HQ_PULSE, INPUT);
  pinMode(P_AMIGA_LMB,      INPUT);
  pinMode(P_AMIGA_RMB,      INPUT);
  pinMode(P_AMIGA_MMB,      INPUT);

  setupMouse();  

  // Blink some more to signal that setup is done.
  blink();
}

void loop() {
  ps2Release();
  byte b1 = ps2Receive();
  short dx = ps2Receive();
  short dy = ps2Receive();
  ps2Inhibit();

  if (b1 &  PS2_MASK_Y_SIGN) {
    dy |= 0xff00;
  } 
  if (b1 &  PS2_MASK_X_SIGN) {
    dx |= 0xff00;
  }
  // orientation is (usually) opposite
  // speed seems to be much slower (usually)
  dy = -dy;

  bool lmb = b1 & PS2_MASK_LMB;
  bool rmb = b1 & PS2_MASK_RMB;
  bool mmb = b1 & PS2_MASK_MMB;
  
  if (lmb) {
    pinMode(P_AMIGA_LMB, OUTPUT);
  } else {
    pinMode(P_AMIGA_LMB, INPUT);
  }

  if (mmb) {
    pinMode(P_AMIGA_MMB, OUTPUT);
  } else {
    pinMode(P_AMIGA_MMB, INPUT);
  }

  if (rmb) {
    pinMode(P_AMIGA_RMB, OUTPUT);
  } else {
    pinMode(P_AMIGA_RMB, INPUT);
  }

  int dirX = sgn(dx);
  int stepsX = abs(dx);
  int dirY = sgn(dy);
  int stepsY = abs(dy);

  // We need to send stepsX horizontal and stepsY vertical signals
  // as quickly as we can. 
  int steps = max(stepsX, stepsY);
  while (steps-- > 0) {
      // Show some activity...
      if (steps % 4 == 0) {
        digitalWrite(LED, digitalRead(LED));
      }
      
      if (stepsY > 0) {
        if (pgm_read_byte(&p[posY])) {
          pinMode(P_AMIGA_V_PULSE, INPUT);
        } else {
          pinMode(P_AMIGA_V_PULSE, OUTPUT);
        }
        if (pgm_read_byte(&pq[posY])) {
          pinMode(P_AMIGA_VQ_PULSE, INPUT);
        } else {
          pinMode(P_AMIGA_VQ_PULSE, OUTPUT);
        }
        posY = (posY + dirY + 4) & 0x3;
        stepsY--;
      }

      if (stepsX > 0) {
        if (pgm_read_byte(&p[posX])) {
          pinMode(P_AMIGA_H_PULSE, INPUT);
        } else {
          pinMode(P_AMIGA_H_PULSE, OUTPUT);
        }
        if (pgm_read_byte(&pq[posX])) {
          pinMode(P_AMIGA_HQ_PULSE, INPUT);
        } else {
          pinMode(P_AMIGA_HQ_PULSE, OUTPUT);
        }
        posX = (posX + dirX + 4) & 0x3;
        stepsX--;
      }

      // 50 micros is just a guess, seems to work out fine.
      delayMicroseconds(50);
  }
}
