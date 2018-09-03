# ESP MQTT JSON Multisensor

Fork from BRUH

### Modifications
- Added MQTT Discovery, no need to configure home assistant
- Changed to no-auth mqtt
- Temperature validation
- Changed to FastLED lib

### Supported Features Include
- **DHT22** temperature sensor
- **DHT22** humidity sensor
- **AM312** PIR motion sensor 
- **photoresistor** or **TEMT600** light sensor
- **RGB led** with support for color, flash, fade, and transition
- **Over-the-Air (OTA)** upload from the ArduinoIDE



### Home Assistant Service Examples
Besides using the card in Home Assistant's user interface, you can also use the Services tool to control the light using the light.turn_on and light.turn_off services. This will let you play with the parameters you can call later in automations or scripts. 

Fade the Light On Over 5 Seconds - light.turn_on
```
{"entity_id":"light.sn1_led",
"brightness":150,
"color_name":"blue",
"transition":"5"
}
```

Flash The Light - light.turn_on
```
{"entity_id":"light.sn1_led",
"color_name":"green",
"brightness":255,
"flash":"short"
}
```

Fade the Light Off Over 5 Seconds - light.turn_off
```
{"entity_id":"light.sn1_led",
"transition":"5"
}
```
