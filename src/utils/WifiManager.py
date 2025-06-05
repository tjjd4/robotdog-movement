import os

class WifiManager:
    WPA_SUPPLICANT_PATH = "/etc/wpa_supplicant/wpa_supplicant.conf"

    @staticmethod
    def configure_wifi(ssid: str, password: str):
        conf_text = f'''
country=TW
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={{
    ssid="{ssid}"
    psk="{password}"
}}
'''
        with open(WifiManager.WPA_SUPPLICANT_PATH, 'w') as f:
            f.write(conf_text)
        print("[WIFI] wpa_supplicant.conf updated")

    @staticmethod
    def reload_wifi():
        os.system("sudo wpa_cli -i wlan0 reconfigure")
        print("[WIFI] wpa_cli reconfigure triggered")

    @staticmethod
    def get_current_ssid():
        try:
            # 取得目前 wlan0 連線的 SSID
            ssid = os.popen("iwgetid -r").read().strip()
            return ssid if ssid else None
        except Exception as e:
            print(f"[WIFI] Error getting current SSID: {e}")
            return None