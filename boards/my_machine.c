/*

  my_machine.c - board init and bit-banged single-wire UART for TMC2209
                 on the MKS Robin Nano V2.1 (STM32F103VET6)

  Part of grblHAL

  Copyright (c) 2026

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

  --------------------------------------------------------------------------

  The MKS Robin Nano V2.x does NOT route the TMC2209 UART lines to a real
  STM32 USART - Marlin talks to them with a bit-banged "software serial" on
  one dedicated GPIO per axis (single wire, half duplex):

      X -> PD5   Y -> PD7   Z -> PD4   E0(A) -> PD9

  This file re-implements that technique for grblHAL, providing the
  tmc_uart_write()/tmc_uart_read() entry points the Trinamic library expects
  (see trinamic/common.h).

  Notes on the implementation:

  - Because each driver has its own dedicated wire, all of them are addressed
    as slave 0 (MS1/MS2 are not used to set addresses on this board). The
    Trinamic plugin defaults to address = motor id, so it is overridden below
    through the driver interface hook.

  - Bit timing uses the Cortex-M3 DWT cycle counter against an absolute
    deadline, so per-bit overhead does not accumulate into drift. At 72 MHz
    and 19200 baud one bit is exactly 3750 cycles.

  - Interrupts are masked per byte rather than per frame, keeping the longest
    blackout around 0.5 ms instead of ~5 ms. Anything longer risks losing
    step timer interrupts.

  - The reply is located by scanning for the sync pattern (0x05, 0xFF, reg)
    rather than assuming a fixed number of leading bytes, which is what
    Marlin's TMCStepper library does. This works whether or not the wiring
    echoes our own transmission back at us.

  NOT YET TESTED ON REAL HARDWARE. If you get "Could not communicate with
  stepper driver" warnings, check with a logic analyzer that the bytes on the
  line look correct, and try nudging UART_BAUD / TMC_REPLY_TIMEOUT_MS.
*/

#include "driver.h"

#if defined(BOARD_MY_MACHINE) && TRINAMIC_ENABLE == 2209

#include <string.h>

#include "motors/trinamic.h"
#include "trinamic/common.h"

// ---- Timing ----------------------------------------------------------

// Marlin's pins_MKS_ROBIN_NANO_V2.h lowers TMC_BAUD_RATE to 19200 on this
// board ("Reduce baud rate to improve software serial reliability"), against a
// software-serial default of 57600 elsewhere. That is the value validated on
// real hardware, so it is the default here too. Override from platformio.ini
// with -D TMC_SOFTUART_BAUD=<rate> if you want to experiment.
//
// The timing itself is baud-agnostic: bit deadlines are precomputed against an
// absolute reference at init, so neither integer truncation nor per-bit code
// overhead accumulates. The practical ceiling comes from the busy-wait loop
// (~8 cycles of sampling granularity, ~17 cycles of start-edge latency) and
// from the TMC2209 auto-detecting the baud rate off the 0x05 sync byte with
// its internal oscillator. Below the limit checked here, the start-edge
// latency stays under ~5% of a bit.
#ifndef TMC_SOFTUART_BAUD
#define TMC_SOFTUART_BAUD 19200
#endif

#if (72000000 / TMC_SOFTUART_BAUD) < 288
#error "TMC_SOFTUART_BAUD is too high for a bit-banged UART on a 72 MHz F1"
#endif

#define UART_BAUD             TMC_SOFTUART_BAUD
#define TMC_REPLY_TIMEOUT_MS  5      // generous - the TMC2209 SENDDELAY defaults
                                     // to 8 bit times, i.e. well under 1 ms

#define N_BIT_DEADLINES 11           // start + 8 data + stop, plus half-bit entry

// Absolute cycle offsets from the frame reference point, rounded rather than
// truncated, so the sampling point never drifts regardless of baud rate.
static uint32_t tx_deadline[N_BIT_DEADLINES]; // edges at 1..10 bit times
static uint32_t rx_deadline[N_BIT_DEADLINES]; // samples at 0.5, 1.5 .. 9.5 bits
static uint32_t us_cycles = 72;

static void timing_init (void)
{
    uint32_t f = SystemCoreClock;

    for (uint_fast8_t n = 0; n < N_BIT_DEADLINES; n++) {
        // (n + 1) bit times, rounded to nearest cycle
        tx_deadline[n] = (uint32_t)(((uint64_t)(n + 1) * 2 * f + UART_BAUD) / (2 * UART_BAUD));
        // (n + 0.5) bit times, rounded to nearest cycle
        rx_deadline[n] = (uint32_t)(((uint64_t)(2 * n + 1) * f + UART_BAUD) / (2 * UART_BAUD));
    }

    us_cycles = f / 1000000UL;
}

static inline void dwt_init (void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t dwt_now (void)
{
    return DWT->CYCCNT;
}

// ---- Per-axis pin table ------------------------------------------------

typedef struct {
    GPIO_TypeDef *port;
    uint32_t pin;
} softuart_pin_t;

// Index matches trinamic_motor_t.axis (0 = X, 1 = Y, 2 = Z, 3 = A/E0).
static const softuart_pin_t uart_pin[] = {
    { GPIOD, 5 }, // X
    { GPIOD, 7 }, // Y
    { GPIOD, 4 }, // Z
#if N_ABC_MOTORS == 1
    { GPIOD, 9 }, // A (E0 header)
#endif
};

#define N_UART_PINS (sizeof(uart_pin) / sizeof(uart_pin[0]))

// ---- Low level bit-bang I/O --------------------------------------------

static void pin_as_output (GPIO_TypeDef *port, uint32_t pin)
{
    GPIO_InitTypeDef cfg = {
        .Mode  = GPIO_MODE_OUTPUT_PP,
        .Pin   = (1u << pin),
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };
    DIGITAL_OUT(port, pin, 1); // set the idle level before switching to output
    HAL_GPIO_Init(port, &cfg);
}

static void pin_as_input (GPIO_TypeDef *port, uint32_t pin)
{
    GPIO_InitTypeDef cfg = {
        .Mode = GPIO_MODE_INPUT,
        .Pin  = (1u << pin),
        .Pull = GPIO_PULLUP, // belt & braces - the TMC2209 also has one internally
    };
    HAL_GPIO_Init(port, &cfg);
}

// Writes one byte, interrupts masked for the duration of the frame only.
static void softuart_write_byte (GPIO_TypeDef *port, uint32_t pin, uint8_t data)
{
    uint32_t t0;

    __disable_irq();

    t0 = dwt_now();

    DIGITAL_OUT(port, pin, 0); // start bit
    while ((uint32_t)(dwt_now() - t0) < tx_deadline[0]);

    for (uint_fast8_t i = 0; i < 8; i++) {
        DIGITAL_OUT(port, pin, data & 1);
        data >>= 1;
        while ((uint32_t)(dwt_now() - t0) < tx_deadline[i + 1]);
    }

    DIGITAL_OUT(port, pin, 1); // stop bit
    while ((uint32_t)(dwt_now() - t0) < tx_deadline[9]);

    __enable_irq();
}

static void softuart_write (GPIO_TypeDef *port, uint32_t pin, const uint8_t *data, uint_fast8_t len)
{
    pin_as_output(port, pin);

    while (len--)
        softuart_write_byte(port, pin, *data++);

    pin_as_input(port, pin);
}

// Waits for a start bit (interrupts enabled) and then samples the byte with
// interrupts masked. Returns false on timeout or on a false start edge.
static bool softuart_read_byte (GPIO_TypeDef *port, uint32_t pin, uint8_t *data, uint32_t timeout_cycles)
{
    uint32_t tw = dwt_now();

    while (DIGITAL_IN(port, pin)) {
        if ((uint32_t)(dwt_now() - tw) > timeout_cycles)
            return false;
    }

    __disable_irq();

    uint32_t t0 = dwt_now();

    // Move to the centre of the start bit and confirm it is still low.
    while ((uint32_t)(dwt_now() - t0) < rx_deadline[0]);

    if (DIGITAL_IN(port, pin)) {
        __enable_irq();
        return false; // glitch, not a real start bit
    }

    uint8_t val = 0;

    for (uint_fast8_t i = 0; i < 8; i++) {
        while ((uint32_t)(dwt_now() - t0) < rx_deadline[i + 1]);
        val |= (DIGITAL_IN(port, pin) ? 1 : 0) << i;
    }

    // Sample past the stop bit but do not hard fail on it.
    while ((uint32_t)(dwt_now() - t0) < rx_deadline[9]);

    __enable_irq();

    *data = val;

    return true;
}

// ---- Trinamic library entry points -------------------------------------

void tmc_uart_write (trinamic_motor_t driver, TMC_uart_write_datagram_t *dgr)
{
    if (driver.axis >= N_UART_PINS)
        return;

    softuart_write(uart_pin[driver.axis].port, uart_pin[driver.axis].pin, dgr->data, sizeof(dgr->data));
}

TMC_uart_write_datagram_t *tmc_uart_read (trinamic_motor_t driver, TMC_uart_read_datagram_t *dgr)
{
    static TMC_uart_write_datagram_t wdgr;

    memset(&wdgr, 0, sizeof(wdgr)); // slave != 0xFF -> caller treats this as a failure

    if (driver.axis >= N_UART_PINS)
        return &wdgr;

    GPIO_TypeDef *port = uart_pin[driver.axis].port;
    uint32_t pin = uart_pin[driver.axis].pin;

    // Send the 4 byte read request.
    softuart_write(port, pin, dgr->data, sizeof(dgr->data));

    // Hunt for the start of the reply frame: 0x05, 0xFF, <register>. Any echo
    // of our own request that happens to be visible on the wire is skipped
    // over by the sliding window, as is any line noise.
    const uint32_t timeout_cycles = TMC_REPLY_TIMEOUT_MS * 1000UL * us_cycles;
    const uint8_t reg = dgr->data[2]; // register byte exactly as we sent it
    uint8_t window[3] = {0};
    uint_fast8_t matched = 0;
    uint32_t tstart = dwt_now();

    while (matched < 3) {

        uint8_t c;

        if (!softuart_read_byte(port, pin, &c, timeout_cycles))
            return &wdgr; // timed out waiting for a byte

        window[0] = window[1];
        window[1] = window[2];
        window[2] = c;

        matched = (window[0] == 0x05 && window[1] == 0xFF && window[2] == reg) ? 3 : 0;

        if ((uint32_t)(dwt_now() - tstart) > timeout_cycles)
            return &wdgr; // no sync pattern in time
    }

    wdgr.data[0] = window[0];
    wdgr.data[1] = window[1];
    wdgr.data[2] = window[2];

    // Remaining 4 payload bytes plus CRC.
    for (uint_fast8_t i = 3; i < sizeof(wdgr.data); i++) {
        if (!softuart_read_byte(port, pin, &wdgr.data[i], timeout_cycles)) {
            memset(&wdgr, 0, sizeof(wdgr));
            break;
        }
    }

    return &wdgr;
}

// ---- Board init ---------------------------------------------------------

// Every driver sits on its own wire, so they all answer to slave address 0.
// Without this the plugin would address Y as 1 and Z as 2 and only X would
// ever reply.
static void on_driver_preinit (motor_map_t motor, trinamic_driver_config_t *config)
{
    config->address = 0;
}

void board_init (void)
{
    static trinamic_driver_if_t driver_if = {
        .on_driver_preinit = on_driver_preinit
    };

    dwt_init();
    timing_init();

    __HAL_RCC_GPIOD_CLK_ENABLE(); // all soft-UART TMC pins (X/Y/Z/A) live on GPIOD

    for (uint_fast8_t i = 0; i < N_UART_PINS; i++)
        pin_as_input(uart_pin[i].port, uart_pin[i].pin); // idle state

    trinamic_if_init(&driver_if);
}

#endif // BOARD_MY_MACHINE && TRINAMIC_ENABLE == 2209
