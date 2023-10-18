/*

Module:  Catena_Iqs620a_simple.ino

Function:
        IQS620A Sensor program for Catena 4610.

Copyright notice:
        This file copyright (C) 2023 by

                MCCI Corporation
                3520 Krums Corners Road
                Ithaca, NY  14850

        See project LICENSE file for license information.

Author:
        Pranau R, MCCI Corporation	October 2023

*/

#include <Catena.h>

#include <Catena_Led.h>
#include <Catena_TxBuffer.h>
#include <Catena_CommandStream.h>
#include <Catena_Mx25v8035f.h>

#include <Wire.h>
#include <hal/hal.h>
#include <mcciadk_baselib.h>

#include <cmath>
#include <type_traits>
#include <MCCI_Catena_Iqs620a.h>
#include <stm32_eeprom.h>

using namespace McciCatena;
using namespace McciCatenaIqs620a;

/****************************************************************************\
|
|   handy constexpr to extract the base name of a file
|
\****************************************************************************/

// two-argument version: first arg is what to return if we don't find
// a directory separator in the second part.
static constexpr const char *filebasename(const char *s, const char *p)
    {
    return p[0] == '\0'                     ? s                            :
           (p[0] == '/' || p[0] == '\\')    ? filebasename(p + 1, p + 1)   :
                                              filebasename(s, p + 1)       ;
    }

static constexpr const char *filebasename(const char *s)
    {
    return filebasename(s, s);
    }

/****************************************************************************\
|
|   Read-only data
|
\****************************************************************************/

static const char sVersion[] = "1.0.0-pre1";

/****************************************************************************\
|
|   VARIABLES
|
\****************************************************************************/

// the primary object
Catena gCatena;

cIQS620A gIQS620A;
bool fProximity;
int16_t sensitivity;
uint16_t rightCounter;
uint16_t leftCounter;
int16_t baseline1;
int16_t baseline2;

//
// the LED
//
StatusLed gLed (Catena::PIN_STATUS_LED);

SPIClass gSPI2(
        Catena::PIN_SPI2_MOSI,
        Catena::PIN_SPI2_MISO,
        Catena::PIN_SPI2_SCK
        );

//  The flash
Catena_Mx25v8035f gFlash;
bool fFlash;

/*

Name:   setup()

Function:
        Arduino setup function.

Definition:
        void setup(
            void
            );

Description:
        This function is called by the Arduino framework after
        basic framework has been initialized. We initialize the sensors
        that are present on the platform, set up the LoRaWAN connection,
        and (ultimately) return to the framework, which then calls loop()
        forever.

Returns:
        No explicit result.

*/

void setup(void)
    {
    gCatena.begin();

    setup_platform();
    setup_flash();

    setup_iqs();
    }

// baseline
void autoZero ()
    {
    gIQS620A.iqsRead();

    baseline1 = gIQS620A.getCh1LTAData();
    baseline2 = gIQS620A.getCh2LTAData();
    gCatena.SafePrintf("baseline1 (LTA) is: %d\n", baseline1);
    gCatena.SafePrintf("baseline2 (LTA) is: %d\n", baseline2);
    }

void setup_platform(void)
    {
#ifdef USBCON
    // if running unattended, don't wait for USB connect.
    if (! (gCatena.GetOperatingFlags() &
        static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fUnattended)))
        {
        while (!Serial)
            /* wait for USB attach */
            yield();
        }
#endif

    gCatena.SafePrintf("\n");
    gCatena.SafePrintf("-------------------------------------------------------------------------------\n");
    gCatena.SafePrintf("This is %s V%s.\n", filebasename(__FILE__), sVersion);
            {
            }
    gCatena.SafePrintf("Enter 'help' for a list of commands.\n");
    gCatena.SafePrintf("(remember to select 'Line Ending: Newline' at the bottom of the monitor window.)\n");

    gCatena.SafePrintf("SYSCLK: %u MHz\n", unsigned(gCatena.GetSystemClockRate() / (1000 * 1000)));

#ifdef USBCON
    gCatena.SafePrintf("USB enabled\n");
#else
    gCatena.SafePrintf("USB disabled\n");
#endif

    Catena::UniqueID_string_t CpuIDstring;

    gCatena.SafePrintf(
        "CPU Unique ID: %s\n",
        gCatena.GetUniqueIDstring(&CpuIDstring)
        );

    gCatena.SafePrintf("--------------------------------------------------------------------------------\n");
    gCatena.SafePrintf("\n");

    // set up the LED
    gLed.begin();
    gCatena.registerObject(&gLed);
    gLed.Set(LedPattern::FastFlash);

    /* find the platform */
    const Catena::EUI64_buffer_t *pSysEUI = gCatena.GetSysEUI();

    uint32_t flags;
    const CATENA_PLATFORM * const pPlatform = gCatena.GetPlatform();

    if (pPlatform)
        {
        gCatena.SafePrintf("EUI64: ");
        for (unsigned i = 0; i < sizeof(pSysEUI->b); ++i)
            {
            gCatena.SafePrintf("%s%02x", i == 0 ? "" : "-", pSysEUI->b[i]);
            }
        gCatena.SafePrintf("\n");
        flags = gCatena.GetPlatformFlags();
        gCatena.SafePrintf(
            "Platform Flags:  %#010x\n",
            flags
            );
        gCatena.SafePrintf(
            "Operating Flags:  %#010x\n",
            gCatena.GetOperatingFlags()
            );
        }
    else
        {
        gCatena.SafePrintf("**** no platform, check provisioning ****\n");
        flags = 0;
        }
    }

void setup_flash(void)
    {
    if (gFlash.begin(&gSPI2, Catena::PIN_SPI2_FLASH_SS))
        {
        fFlash = true;
        gFlash.powerDown();
        gCatena.SafePrintf("FLASH found, put power down\n");
        }
    else
        {
        fFlash = false;
        gFlash.end();
        gSPI2.end();
        gCatena.SafePrintf("No FLASH found: check hardware\n");
        }
    }

void setup_iqs()
    {
    Wire.begin();
    delay(100);

    if(!gIQS620A.begin())
        {
        gCatena.SafePrintf("No IQS620A Sensor found: check wiring\n");
        fProximity = false;
        }
    else
        {
        gCatena.SafePrintf("IQS620A Sensor found!\n");
        rightCounter = 0;
        leftCounter = 0;
        sensitivity = 100;
        fProximity = true;
        // autoZero();
        }
    }

uint32_t gRebootMs;

void loop()
    {
    gCatena.poll();
    gIQS620A.iqsRead();

    // baseline
    autoZero();

    // Raw Count
    int16_t Ch1Data = gIQS620A.getCh1RawData();
    int16_t Ch2Data = gIQS620A.getCh2RawData();

    gCatena.SafePrintf("Channel 1 raw count: %d", Ch1Data);
    gCatena.SafePrintf("\t\tChannel 2 raw count: %d", Ch2Data);

    if ((baseline1 - Ch1Data) > sensitivity && (baseline2 - Ch2Data) > sensitivity)
    // if ((Ch1Data - baseline1) < sensitivity && (Ch2Data - baseline2) < sensitivity)
        {
        rightCounter = rightCounter + 1;
        leftCounter = leftCounter + 1;
        }
    else if ((baseline1 - Ch1Data) > sensitivity)
        {
        rightCounter = rightCounter + 1;
        }
    else if ((baseline2 - Ch2Data) > sensitivity)
        {
        leftCounter = leftCounter + 1;
        }
    else
        {
        }

    gCatena.SafePrintf("\t\tRight Counter: %d", rightCounter);
    gCatena.SafePrintf("\t\tLeft Counter: %d", leftCounter);
    gCatena.SafePrintf("\n");
    delay(500);
    }