# STM32 Drum Machine

## Description
The goal of this project was to create a drum machine on an STM32 that is capable of playing and mixing different sounds, in addition to recording and playing back user inputs.

## Components
This code was created for an STM32 Nucleo F446ZE, which has a built in DAC. The circuit includes an amplifier, potentiometer and push buttons. The project can output 5 unique drum sounds, and includes a button for recording and replaying.

## Method
### Outputting Audio - Timer & DAC
The audio is output at a rate of roughly 22050 Hz through the DAC. To accomplish this, a timer triggers the DAC to output any of its contents about once every 45 microseconds, which gives about 22050 outputs every second. The timer chosen has a default clock speed of 84 MHz on the F446ZE. The counter period is then given by the ratio of the two frequencies, which comes out to about 3809. To actually update what is output through the DAC, the infinite loop updates the contents of the DAC while the timer is incrementing. The system clock runs at 168 MHz, and there are no time-consuming operations so a whole run of the loop should be done MUCH faster than 22050 Hz, so there should be no problems with outputting every sample reliably. The actual samples themselves are 16 bit integer arrays stored in the flash memory of the STM32 so they can be quickly accessed. The F446ZE has 512KB of flash, which is more than enough for 5 sounds. 
### Outputting Audio - Loop Details
At the beginning of the infinite loop, the state of the each button is read (pressed or unpressed). Some logic is done as a form of debouncing. An array keeps track of the number of samples left to play for each sound. When a button is pressed, the number of samples left to play for that sound is set to its full value, meaning the whole file is unplayed. Then the code will check which sounds have any samples still unplayed, and sum the values of the current sample for all these sounds so the result is mixed together (e.g., if two buttons are pressed at the same time the sounds are mixed together). The number of samples left to play for these sounds is then decremented, meaning there is one less sample left to play. When all of the samples in a sound have been played, the number of unplayed samples is zero, so nothing for that sound will be output until the button is pressed and the value is reset again. Finally, the value to be output in the DAC is set as the resulting mix of samples. Since the DAC for the F446ZE is 12 bit, the 16 bit sample is shifted right so as to discard to 4 least significant bits. Then the loop waits until the timer has reset to restart - this is done by waiting in a while loop well checking the value of a global boolean updated in the timer callback function.
### Recording and playback
There are three states that the loop can be in: Default, Record, and Play. In Default mode, it works as described above. If the user presses the Record button, then the program enters Record mode. Now when a button is pressed which button was pressed and when it was pressed is recorded. To record the time it was pressed the loop iteration is counted. There are millions of loops every second, so this number gets very large, but storing it in a 64 bit integer means it will never overflow in a practical sense. In Play mode, the recording is played back on a loop. This works by checking if the loop iteration has reached one of the recorded ones, and if so assigning a button press to the button that was pressed at that moment. Then the sound is output as if the user had pressed the button. Checks are also implemented to ensure that a user can only enter Record and Play mode while they are in Default mode.

### Circuitry
To output the sound, the DAC output is fed through a potentiometer and an LM386 amplifier following this [guide](https://www.circuitbasics.com/build-a-great-sounding-audio-amplifier-with-bass-boost-from-the-lm386/).  
Not shown here, there is also a 100nF capacitor and 10Ω resistor in series connected to pin 5 output and ground before the speaker.
![Circuit](https://github.com/lj-watson/stm32-project/blob/master/STM32_Circuit.png)

## Acknowledgements
* Developed with [ethanluan10](https://github.com/ethanluan10)
* [spanceac's project](https://github.com/spanceac) for the sound bytes
