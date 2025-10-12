# menuconfig
勾选websocket server support

# Tips
1. 在AP+STA模式下，尽量不要使用esp_wifi_stop()，会导致代码core，只需要执行esp_wifi_disconnect()就可以。