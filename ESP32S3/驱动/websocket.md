# menuconfig
勾选websocket server support

# Tips
1. 在AP+STAmo
2. 执行esp_wifi_stop()，必须要执行esp_wifi_start()，都则操作wifi导致代码core。正常情况下只需要执行esp_wifi_disconnect()就可以