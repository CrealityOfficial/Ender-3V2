# Ender-3 V2 Firmware

Creality attaches great importance to users. As 3D printing industry evangelist, Creality, dedicating to bringing benefits to human beings via technology innovations, has received support from both users and 3D printing enthusiasts. With gratefulness, Creality wants to continue the pace of making the world a better place with you all. This time, Creality will open the source code and we believe GitHub is the way to go. 

This is the repository that contains the source code and the development versions of the firmware running on the Creality [Ender-3 V2](https://www.creality.com/goods-detail/ender-3-v2-3d-printer), [Ender-3 S1](https://www.creality.com/goods-detail/creality-ender-3-s1-3d-printer), [CR-30/3DPrintMill](https://www.creality.com/goods-detail/creality-3dprintmill-3d-printer), [Ender-2 Pro](https://www.creality.com/goods-detail/creality-ender-2-pro-3d-printer), [Sermoon D1](https://www.creality.com/goods-detail/creality-sermoon-d1-3d-printer) and more products in the future. It's based on the well-known Marlin but with modifications.

The firmware for the Creality Ender-3 V2is proudly based on Marlin2.0 byScott Lahteine (@thinkyhead), Roxanne Neufeld (@Roxy-3D), Chris Pepper (@p3p), Bob Kuhn (@Bob-the-Kuhn), João Brazio (@jbrazio), Erik van der Zalm (@ErikZalm) and is distributed under the terms of the GNU GPL 3 license.

If you want to download the latest firmware version, go to Releases page and download the needed files. In the releases page you will find the source code and the SD Files needed for the LCD Display. After that, normally you need to update the SD files of the display and gradually complete the updates of menus, graphics and functionalities. 

Please refer to: [YouTube](https://www.youtube.com/watch?v=Jswzrh2_ekk)
In order to get instructions on how to upgrade the firmware and load new LCD SD files to the display.

# Table of contents
Windows build 
Documentation
Please refer to: [Marlin Page](https://marlinfw.org/docs/basics/introduction.html)

# New Features
1. Fix the problem of losing long file names in resume printing.
2. Fix the problem of time reset with automatic return to zero.
3. Fix some UI display defects.
4. Add auto zero return function and UI display after power up.
5. Add auto screen resting function.
6. Add card pulling detection function.


# Marlin 3D Printer Firmware

### Supported Platforms

  Platform|MCU|Example Boards
  --------|---|-------
  [STM32F103](https://www.st.com/en/microcontrollers-microprocessors/stm32f103.html)|ARM® Cortex-M3|Creality4.2.2,4.2.3,4.2.7,4.3.1,4.2.5,4.2.10
  [GD32F303](https://www.st.com/en/microcontrollers-microprocessors/stm32f103.html)|ARM® Cortex-M3|Creality4.2.2,4.2.3,4.2.7,4.3.1,4.2.5,4.2.10
  [STM32F401](https://www.st.com/en/microcontrollers-microprocessors/stm32f401.html)|ARM® Cortex-M4|ARMED, Rumba32, SKR Pro, Lerdge, FYSETC S6

# Issues and Suggestions
Your feedback is very important to us, as it helps us improve even faster. Feel free to feedback us if there is an issue.
In order to get responses in an efficient way, we recommend you to follow some guidelines:
1. First of all, search for related issues.
2. Detail the firmware version you're running.
3. Explain to us the error or bug, so that we can test it properly.
4. In the title, indicate the label of the issue. (For example: #issue)

# Development Process
The code is currently in development, trying to improve functionalities.
Since it’s possible for the advanced users to contribute in firmware development, we suppose you know the points even if they have not been clearly illustrated by Creality.

The master branch is stable and it's currently of the version 2.0.x. The master branch stores code created by Creality. Once a release is done, the users, get to upgrade the version and give feedback to us. We get to know the bugs as well as optimization based on the feedback from you and Creality will make a decision on what to be included into the master branch. 

By integrating suggested improvements, we will make a branch from the version.

This is a classic code development process and we want more, so we really want you to participate from the very beginning.
