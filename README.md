# RC Current Sensor firmware
This is the firmware for my 60A RC current sensor design. The hardware design can be found here: [github](https://github.com/bodri/kapuki-cursen-hw)
and you can read more about this project in my [blog](https://www.bodrico.com/2020/03/rc-current-sensor.html).

![Main telemetry screen](images/telemetry.png)

# Programming your device
This is a [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) project. You can use the IDE directly to program your device
or simply use the [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) to falsh the provided [hex](https://github.com/bodri/kapuki-cursen-fw/blob/master/kapuki-cursen-fw.hex).

If you use the IDE and you add some features please create a pull-request here that we can share with everyone.

# Using the sensor
Connect the sensor to a JETI EX receiver pin which is set to use EX Bus protocol, like the pin 3 on my REX3 receiver:

![Jeti transmitter settings](images/settings.png)

That's it folks. 
