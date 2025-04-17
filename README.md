# 🚀 Embedded Driver Development - STM32 Bare-Metal

This repository is a personal implementation and exploration of **bare-metal driver development for STM32 microcontrollers**, inspired by the "Mastering Microcontroller: Embedded Driver Development (MCU1)" course. The entire project focuses on writing **peripheral drivers from scratch** in **Embedded C**, without using HAL or any third-party libraries.

🧠 Learn how drivers really work under the hood by diving into **register-level programming** with STM32F4 series microcontrollers.

---

## 📚 What You'll Find in This Repo

### ✅ Fully Working Peripheral Drivers for:
- **GPIO** – Input/output modes, alternate functions
- **SPI** – Full duplex, command handling, interrupts
- **I2C** – Master/Slave modes, interrupt-driven communication
- **USART (UART)** – Transmit, receive, baud rate config, interrupts

### 📂 Application Examples Included:
The `src/` folder and project subdirectories contain various test applications demonstrating how to use each driver in real use cases.

| File/Directory                   | Description                                      |
|----------------------------------|--------------------------------------------------|
| `001HelloWorld/`                | Initial test project                             |
| `002SampleApp/`                 | GPIO examples (button/LED)                       |
| `004PeriClockEnable/`          | Peripheral clock enable testing                  |
| `005HSI_Measurement/`          | Clock configuration & HSI measurement            |
| `Clock/`                        | Clock source config & AHB/APB bus experiments    |
| `stm32f4xx_drivers/drivers`           | Driver source code for GPIO, SPI, I2C, USART     |
| `stm32f4xx_drivers/src/001led_toggle.c`          | Toggle LED with GPIO                             |
| `stm32f4xx_drivers/src/002led_button.c`          | Button-controlled LED                            |
| `stm32f4xx_drivers/src/006spi_tx_testing.c`      | SPI data transmit                                |
| `stm32f4xx_drivers/src/007spi_txonly_arduino.c`  | SPI Tx-only example with Arduino                 |
| `stm32f4xx_drivers/src/008spi_cmd_handling.c`    | SPI command/response protocol                    |
| `stm32f4xx_drivers/src/009spi_message_rcv_it.c`  | SPI interrupt-based message RX                   |
| `stm32f4xx_drivers/src/010i2c_master_tx_testing.c`| I2C Master TX example                           |
| `stm32f4xx_drivers/src/011i2c_master_rx_testing.c`| I2C Master RX example                           |
| `stm32f4xx_drivers/src/012i2c_master_rx_testingIT.c`| I2C RX via Interrupts                        |
| `stm32f4xx_drivers/src/013i2c_slvae_tx_srting.c` | I2C Slave TX with string                         |

---

## 🛠️ What This Project Teaches

- How to **write driver headers**, define API prototypes, and implement them
- How to extract **register-level info** from STM32 **datasheets** and **reference manuals**
- How to work with **IRQ handling**, **NVIC**, and **vector tables**
- Understanding **clock trees**, **AHB/APB buses**, **PLL**, and **baud rate calculations**
- Debugging protocols with **logic analyzers** and interpreting traces
- Building **application-level test code** from scratch

---

## 🧰 Tools & Setup

- **MCU**: STM32F407 Discovery Board (Cortex-M4)
- **IDE**: STM32CubeIDE or Eclipse + GCC Toolchain
- **Compiler**: `arm-none-eabi-gcc`
- **Debugger/Flasher**: ST-Link, OpenOCD
- **Optional**: Logic analyzer (for SPI/I2C protocol analysis)

---

## 🔄 No HAL. No Libraries. Just You and the Datasheet.

> This project is built entirely without STM32 HAL/LL libraries.  
> Every peripheral is controlled through **direct register manipulation** based on **reference manual** and **datasheet insights**.
---

## 🤝 Contribution

Want to add new examples or ports to other MCUs? PRs are welcome!  
This project is for learning — feel free to fork, modify, and experiment.

---


## 👋 Acknowledgements

- Inspired by FastBit Embedded Brain Academy
- Special thanks to those pushing for deep understanding of embedded systems

---

> “Don’t just blink an LED. Understand *how* the LED blinks.”  
> — Every Embedded Dev, Eventually 😄
