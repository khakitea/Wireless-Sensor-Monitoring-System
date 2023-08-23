# Wireless-Sensor-Monitoring-System-ADC-DMA
Engine Capstone project for NAVSEA and University of Washington

This code is for the sensors that require ADC DMA.
Voltage, current and pressure sensors.

Configurations through Cube MX can be found in "ADC_DMA_multichannel.ioc"
> Channels 7-10 are enabled
> ADC1 enabled and NVIC for interrupt
> ADC1 added to DMA (mode: circular, memory data width: word)
> Param settings were slightly changed


For code content, 
Core > Src > "main.c"
