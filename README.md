# Kira

## Upload arduino's code

1. ``` cd Documents/Kira/src/cpp ```
2. ``` ./HelloWorld -u ```

## To start motors

``` ./HelloWorld speedG speedD ```

## to open arduino's screen

``` sudo screen /dev/ttyUSB0 9600 ```

## Warning!!

Before uploading arduino's code make sure that nothing is using /dev/tty/USB0 port.
By running in terminal ``` sudo fuser /dev/ttyUSB0 ```.
If anything is using it, you should use ``` sudo kill <pid> ``` <pid> est à modifier selon la valeur du pid.
