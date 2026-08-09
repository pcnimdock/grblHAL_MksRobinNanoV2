/*
  my_machine_map.h - board map for MKS Robin Nano V2.1 (STM32F103VET6)

  Part of grblHAL

  Pin assignments extracted from the working Marlin 2.1.2 configuration for
  MOTHERBOARD = BOARD_MKS_ROBIN_NANO_V2, see:
  Marlin/src/pins/stm32f1/pins_MKS_ROBIN_NANO_V2.h

  X/Y/Z use TMC2209 drivers in single-wire UART mode, one dedicated GPIO
  per axis (NOT routed to a hardware USART peripheral) -> handled here with
  a bit-banged software UART, see boards/my_machine.c

  Spindle control is ON/OFF only, no PWM. The output is PB0, which is the
  connector silkscreened HE1 on the board (the schematic calls that net
  HEATER2 - do not let that mislead you; HE0 / schematic net HEATER1 is PC3
  and is NOT the pin used here).

  Endstop / probe wiring (confirmed against the working Marlin config, which
  has Z_HOME_DIR = +1 and USE_ZMAX_PLUG):

      X limit  -> PA15  (X- header)
      Y limit  -> PA12  (Y- header)
      Z limit  -> PC4   (Z+ header)  Z homes UP, to max
      Probe    -> PA11  (Z- header)  PCB continuity probe, G38.x

  All four are active low (Marlin's *_ENDSTOP_INVERTING true), which is
  grblHAL's default -> leave $5=0.

  Per the MKS schematic (ENDstops.SchDoc) each of these MCU pins is a shared
  node: the endstop connector AND the corresponding TMC2209 DIAG line, bridged
  by an unpopulated jumper (J21..J25 / header J20). PA15 is X-0 and X-DIAG,
  PA12 is Y-0 and Y-DIAG, PA11 is Z-0 and Z-DIAG, PC4 is Z+0 and E0-DIAG.
  Those jumpers are open from the factory, so the pins carry only the switches
  - which is why sensorless homing must stay off: it would need the same pins.

  This board has NOT been tested on real hardware yet - verify polarity of
  limit switches ($5), enable pins ($4) and stepper direction ($3) once
  flashed, and watch the console for "Could not communicate with stepper
  driver" warnings which would indicate the soft-UART timing needs tuning.
*/

#ifndef STM32F103xE
#error "This board has a STM32F103VET6 processor, select a corresponding build!"
#endif

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "MKS Robin Nano V2.1"
#define BOARD_URL "https://github.com/makerbase-mks/MKS-Robin-Nano-V2.X"

// Without this the driver reports "STM32F103RC" in $I - both the RC and the VE
// are high-density parts and share stm32f103xe.h, so it cannot tell them apart.
#define MCU_NAME "STM32F103VE"

// The PCB probe is wired to the Z- header (PA11), see AUXINPUT0 below.
#ifdef PROBE_ENABLE
#undef PROBE_ENABLE
#endif
#define PROBE_ENABLE 1

#define HAS_BOARD_INIT

// TMC2209 UART is bit-banged in software, one pin per axis - see my_machine.c
#ifdef TRINAMIC_ENABLE
#undef TRINAMIC_ENABLE
#endif
#ifdef TRINAMIC_MIXED_DRIVERS
#undef TRINAMIC_MIXED_DRIVERS
#endif
#define TRINAMIC_ENABLE 2209
#define TRINAMIC_MIXED_DRIVERS 0

// Define step pulse output pins.
// NOTE: steps are on different GPIO ports per axis -> GPIO_BITBAND required.
#define X_STEP_PORT              GPIOE
#define X_STEP_PIN                3   // PE3
#define Y_STEP_PORT               GPIOE
#define Y_STEP_PIN                0   // PE0
#define Z_STEP_PORT               GPIOB
#define Z_STEP_PIN                5   // PB5
#define STEP_OUTMODE              GPIO_BITBAND

// Define step direction output pins.
#define X_DIRECTION_PORT          GPIOE
#define X_DIRECTION_PIN            2   // PE2
#define Y_DIRECTION_PORT          GPIOB
#define Y_DIRECTION_PIN            9   // PB9
#define Z_DIRECTION_PORT          GPIOB
#define Z_DIRECTION_PIN            4   // PB4
#define DIRECTION_OUTMODE          GPIO_BITBAND

// Define stepper driver enable/disable output pins (active low on this board).
#define X_ENABLE_PORT             GPIOE
#define X_ENABLE_PIN               4   // PE4
#define Y_ENABLE_PORT             GPIOE
#define Y_ENABLE_PIN               1   // PE1
#define Z_ENABLE_PORT             GPIOB
#define Z_ENABLE_PIN               8   // PB8

// Define homing/hard limit switch input pins.
// X and Y are on GPIOA (X-/Y- headers), Z is on the Z+ header because Z homes
// towards max - PA11 (Z-) is used for the probe instead, see AUXINPUT0.
#define LIMIT_PORT                GPIOA  // default for axes without an explicit port
#define X_LIMIT_PORT              GPIOA
#define X_LIMIT_PIN               15   // PA15, X- header
#define Y_LIMIT_PORT              GPIOA
#define Y_LIMIT_PIN               12   // PA12, Y- header
#define Z_LIMIT_PORT              GPIOC
#define Z_LIMIT_PIN                4   // PC4,  Z+ header (Z homes up)
#define LIMIT_INMODE               GPIO_BITBAND

// Optional 4th axis (A), wired to the E0 stepper header.
// Uncomment the block below AND set N_ABC_MOTORS=1 (N_AXIS=4) in my_machine.h
// to use it.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT               GPIOD
#define M3_STEP_PIN                 6   // PD6
#define M3_DIRECTION_PORT          GPIOD
#define M3_DIRECTION_PIN            3   // PD3
#define M3_ENABLE_PORT             GPIOB
#define M3_ENABLE_PIN               3   // PB3
#endif

// Spindle relay, ON/OFF only. PB0 = the HE1 screw terminal on J13 (Marlin's
// HEATER_1_PIN; the schematic labels this net HEATER2).
// NOTE: the driver does support hardware PWM on this exact pin (PB0 is
// TIM1_CH2N, see the table in Inc/driver.h) if you ever fit a spindle
// controller with a PWM input - it would need no rewiring, just the
// SPINDLE_PWM_PORT/PIN defines. Pointless while PB0 drives a relay.
#define AUXOUTPUT0_PORT            GPIOB // Spindle enable
#define AUXOUTPUT0_PIN              0    // PB0

// Flood coolant on the FAN mosfet (J12, Marlin's FAN0_PIN). Harmless if nothing
// is wired there - it just gives you M8/M9. Comment out the COOLANT_FLOOD_*
// lines below if you would rather keep the pin free.
#define AUXOUTPUT1_PORT            GPIOB // Coolant flood
#define AUXOUTPUT1_PIN              1    // PB1

#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT        AUXOUTPUT0_PORT
#define SPINDLE_ENABLE_PIN         AUXOUTPUT0_PIN
#endif

#define COOLANT_FLOOD_PORT         AUXOUTPUT1_PORT
#define COOLANT_FLOOD_PIN          AUXOUTPUT1_PIN

// PCB probe on the Z- header. Marlin reaches this via NOZZLE_AS_PROBE plus
// Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN; in grblHAL it becomes the G38.x probe
// input. Active low, so leave $6=0.
#define AUXINPUT0_PORT             GPIOA // Probe
#define AUXINPUT0_PIN              11    // PA11, Z- header

#if PROBE_ENABLE
#define PROBE_PORT                 AUXINPUT0_PORT
#define PROBE_PIN                  AUXINPUT0_PIN
#endif

// No safety door / reset / feed hold / cycle start wired on this build -
// grblHAL works fine without them (use serial real-time commands instead).
//
// HARDWARE E-STOP on the MT_DET2 header (PE6). EXTI line 6 is unused, and the
// board already provides the conditioning: R36 pulls the net to +3V3, R37 is in
// series and C32 filters to ground (MKS schematic, P-AUX sheet), so a bare
// switch needs no extra components.
//
// Wire a NORMALLY CLOSED button between the signal pin and GND. Idle = closed =
// LOW = not triggered; pressed, or the cable cut, = open = pulled HIGH =
// triggered. That is why $14 must stay 0: grblHAL reads the raw pin and treats
// HIGH as asserted, so a broken wire trips the e-stop instead of silently
// disabling it.
//
// Do NOT use the MT_DET1 header (PA4) for this - PA4 shares EXTI line 4 with
// the Z limit on PC4, and only one port can drive a given EXTI line.
//
#define AUXINPUT1_PORT             GPIOE // E-stop, MT_DET2 header
#define AUXINPUT1_PIN               6    // PE6

// With ESTOP_ENABLE (which defaults to 1 at COMPATIBILITY_LEVEL 0) grblHAL maps
// RESET_PIN to the e_stop signal rather than a soft reset: protocol_main_loop()
// raises Alarm_EStop and blocks everything until the button is released.
#define RESET_PORT                 AUXINPUT1_PORT
#define RESET_PIN                  AUXINPUT1_PIN

#if SDCARD_ENABLE
// Onboard SD (SDIO) is NOT supported by this map - only add SPI SD card
// support if you wire a card reader to unused pins.
#error "SD card is not mapped for this board yet - disable SDCARD_ENABLE."
#endif