> Author  
> [Safouene Mahfoudh](https://github.com/Safouene-Mahfoudh)  

According to the datasheet of hc-sr04, the following is required to be done :

* Keep the Trig pin HIGH for at least 10us
* The Module will now send 8 cycle burst of ultrasound at 40 kHz and detect whether there is a pulse signal back
* IF the signal returns, module will output a HIGH PULSE whose width will be proportional to the range of the object.
* Distance can be calculated by using the following formula :- range = high level time * velocity (340m/s) / 2
* We can also use uS / 58 = Distance in cm or uS / 148 = distance in inch
* It is recommended to wait for at least 60ms before starting the operation again.

