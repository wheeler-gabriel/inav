/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"


#ifdef USE_GAS_SENSOR

#include "common/utils.h"
#include "drivers/time.h"
#include "io/serial.h"
#include "sensors/gas.h"
#include "build/debug.h"

/*
 * Frame format (4 bytes, 115200 8N1):
 *   byte 0: header 1  (0xA5)
 *   byte 1: header 2  (0x5A)
 *   byte 2: status    (0 = no gas, 1 = gas detected)
 *   byte 3: checksum  (status XOR 0xFF)
 * Status and checksum can never contain the header sequence, so the
 * parser cannot lock onto the middle of a frame.
 */
#define GAS_SENSOR_BAUD     115200
#define GAS_HEADER_1        0xA5
#define GAS_HEADER_2        0x5A
#define GAS_FRAME_SIZE      4
#define GAS_TIMEOUT_MS      1000

typedef struct gasSensor_s {
    bool hasData;
    bool detected;
    timeMs_t lastDetectionMs;
    timeMs_t lastFrameMs;
} gasSensor_t;

static serialPort_t *gasPort;
static uint8_t buf[GAS_FRAME_SIZE];
static uint8_t bufPos;
static gasSensor_t gas;


bool gasInit(void)
{
    const serialPortConfig_t *cfg = findSerialPortConfig(FUNCTION_GAS_SENSOR);
    if (!cfg) {
        return false;
    }

    gasPort = openSerialPort(cfg->identifier, FUNCTION_GAS_SENSOR, NULL, NULL,
                             GAS_SENSOR_BAUD, MODE_RX, SERIAL_NOT_INVERTED);

    return gasPort != NULL;
}

static bool gasFrameValid(const uint8_t *f)
{
    return f[2] <= 1 && (uint8_t)(f[2] ^ 0xFF) == f[3];
}

static void gasProcessFrame(const uint8_t *f)
{
    const bool detected = f[2] != 0;
    const timeMs_t now = millis();

    if (detected && !gas.detected) {
        gas.lastDetectionMs = now;
    }
    gas.detected = detected;
    gas.lastFrameMs = now;
    gas.hasData = true;

    DEBUG_SET(DEBUG_ALWAYS, 0, detected);
    DEBUG_SET(DEBUG_ALWAYS, 1, f[2]);

    DEBUG_SET(DEBUG_ALWAYS, 2, gas.lastDetectionMs);
    if(debugMode == DEBUG_ALWAYS){
    debug[3]++;

    }
}
void gasUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!gasPort) {
        return;
    }

    while (serialRxBytesWaiting(gasPort)) {
        const uint8_t c = serialRead(gasPort);

        if (debugMode == DEBUG_ALWAYS) {
            debug[5]++;                 // raw bytes received
            debug[7] = (int32_t)(((uint32_t)debug[7] << 8) | c);  // last 4 raw bytes, read as hex
        }

        // resync on the two-byte header
        if (bufPos == 0 && c != GAS_HEADER_1) {
            continue;
        }
        if (bufPos == 1 && c != GAS_HEADER_2) {
            bufPos = (c == GAS_HEADER_1) ? 1 : 0;
            continue;
        }

        buf[bufPos++] = c;

        if (bufPos == GAS_FRAME_SIZE) {
            if (debugMode == DEBUG_ALWAYS) {
                debug[4]++;             // frames with valid A5 5A header
                debug[6] = (int32_t)(((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                                     ((uint32_t)buf[2] << 8) | buf[3]);   // last framed bytes
            }
            if (gasFrameValid(buf)) {
                gasProcessFrame(buf);
            }
            bufPos = 0;
        }
    }
}

bool isGasSensorEnabled(void)
{
    return gasPort != NULL;
}

bool isGasSensorHealthy(void)
{
    return gasPort && gas.hasData && (millis() - gas.lastFrameMs) < GAS_TIMEOUT_MS;
}

bool isGasDetected(void)
{
    return isGasSensorHealthy() && gas.detected;
}

timeMs_t gasGetLastDetectionMs(void)
{
    return gas.lastDetectionMs;
}

#endif // USE_GAS_SENSOR
