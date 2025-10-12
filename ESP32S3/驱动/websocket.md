# menuconfig
勾选websocket server support

# Tips
1. 在AP+STA模式下，执行执行esp_wifi_stop()后，再执行esp_wifi_set_mode()和esp_wifi_set_config()回core，但是如果在STA模式下执行esp_wifi_stop()，再执行esp_wifi_set_mode()和esp_wifi_set_config()则不会core。因此尽量不要执行执行执行esp_wifi_stop()，执行执行执行esp_wifi_disconnect()即可。