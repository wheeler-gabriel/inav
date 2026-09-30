#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "io/serial.h"
#include "drivers/time.h"

#if defined(USE_RANGEFINDER_US100)
#include "drivers/rangefinder/rangefinder_virtual.h"

#define US100_BAUDRATE           9600
#define US100_TRIGGER_BYTE       0x55
#define US100_PACKET_SIZE        2
#define US100_TRIGGER_INTERVAL_MS 120   // datasheet: >100ms between triggers
#define US100_REPLY_TIMEOUT_MS   60     // give up on a stale/partial reply
#define US100_KEEP_DATA_TIMEOUT_MS 500

static serialPort_t * serialPort = NULL;
static serialPortConfig_t * portConfig;

static uint8_t  lineBuf[US100_PACKET_SIZE];
static uint8_t  bufIndex = 0;
static bool     waitingForReply = false;
static timeMs_t lastTriggerMs = 0;

static bool     hasNewData = false;
static bool     hasEverData = false;
static int32_t  sensorData = RANGEFINDER_NO_NEW_DATA;
static timeMs_t lastValidReadingMs;

static bool us100RangefinderDetect(void)
{
    portConfig = findSerialPortConfig(FUNCTION_RANGEFINDER);
    return portConfig != NULL;
}

static void us100RangefinderInit(void)
{
    if (!portConfig) {
        return;
    }

    serialPort = openSerialPort(portConfig->identifier, FUNCTION_RANGEFINDER, NULL, NULL,
                                 US100_BAUDRATE, MODE_RXTX, SERIAL_NOT_INVERTED);
}

static void us100RangefinderUpdate(void)
{
    if (!serialPort) {
        return;
    }

    const timeMs_t now = millis();

    // Timed out waiting for a reply to the last trigger - drop it and flush any late/partial bytes
    if (waitingForReply && (now - lastTriggerMs) > US100_REPLY_TIMEOUT_MS) {
        waitingForReply = false;
        bufIndex = 0;
        while (serialRxBytesWaiting(serialPort) > 0) {
            serialRead(serialPort);
        }
    }

    // Only accumulate bytes while we're actually expecting a reply
    if (waitingForReply) {
        while (waitingForReply && serialRxBytesWaiting(serialPort) > 0) {
            lineBuf[bufIndex++] = serialRead(serialPort);

            if (bufIndex == US100_PACKET_SIZE) {
                const uint16_t distanceMm = ((uint16_t)lineBuf[0] << 8) | lineBuf[1];
                bufIndex = 0;
                waitingForReply = false;

                // 0xFFFF / 0 => no valid echo per datasheet; otherwise sensor range is ~2-450cm
                if (distanceMm > 0 && distanceMm <= 4500) {
                    sensorData = distanceMm / 10; // mm -> cm
                    hasNewData = true;
                    hasEverData = true;
                    lastValidReadingMs = now;
                }
            }
        }
    }

    // Fire the next trigger once the throttle interval has elapsed and we're not already waiting
    if (!waitingForReply && (now - lastTriggerMs) >= US100_TRIGGER_INTERVAL_MS) {
        serialWrite(serialPort, US100_TRIGGER_BYTE);
        lastTriggerMs = now;
        waitingForReply = true;
    }
}

static int32_t us100RangefinderGetDistance(void)
{
    if (hasNewData) {
        hasNewData = false;
        return sensorData;
    }

    if ((millis() - lastValidReadingMs) < US100_KEEP_DATA_TIMEOUT_MS) {
        return sensorData;
    }

    return hasEverData ? RANGEFINDER_OUT_OF_RANGE : RANGEFINDER_NO_NEW_DATA;
}

virtualRangefinderVTable_t rangefinderUS100Vtable = {
    .detect = us100RangefinderDetect,
    .init = us100RangefinderInit,
    .update = us100RangefinderUpdate,
    .read = us100RangefinderGetDistance
};

#endif
