# ESP_Thermal_Camera_WebServer

Thermal Camera based on: <br>
1) NodeMCU ESP32S
2) MLX 90641 Thermal Camera 12x16 pixels
3) ESPAsyncWebServer
4) SPIFFS - file system 
<br>

Important features - TO DO LIST:
- [x] Grabbing thermal image from MLX90641
- [x] Build WebServer with ESPAsyncWebServer
- [x] Automatic update of variables (e.g. MaxTemp) on Website
- [x] Save thermal image as picture (BMP) in SPIFFS
- [x] Show Thermal Image (BMP) on Website
- [x] Automatic update of BMP (suitable SetInterval in <script>)
- [ ] Case - 3D Model and print it on FDM 3D Printer
- [ ] Tweak updating Thermal Image (now SetInterval set to 1 sec - buggy image if set <1sec or there is more Clients)
- [ ] Reducing overall latency (Issue #2)
- [ ] Maybe Stream thermal image to Website (because why not) - faster updating
