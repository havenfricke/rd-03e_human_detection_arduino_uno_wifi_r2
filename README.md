## Circuit
- VCC -> 5V
- GND -> GND
- OT1 -> 0 RX
- RX  -> 1 TX

## For Deployment
Remove ```!Serial```
```
void setup() {
  Serial.begin(115200);
  delay(200);          // small startup delay only
  Serial1.begin(250000);
}
```
