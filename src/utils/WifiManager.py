import os
import subprocess

class WifiManager:
    NETWORK_MANAGER_CONFIG_PATH = "/etc/NetworkManager/system-connections/"

    @staticmethod
    def add_wifi_config(ssid: str, password: str):
        try:
            subprocess.run(
                [
                    "sudo", "nmcli", "connection", "add",
                    "type", "wifi",
                    "con-name", ssid,
                    "ssid", ssid,
                    "wifi-sec.key-mgmt", "wpa-psk",
                    "wifi-sec.psk", password
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True
            )
        except subprocess.CalledProcessError as e:
            print(f"[WIFI] Error adding Wifi configuration {ssid}: {stderr}")

    @staticmethod
    def rm_wifi_config(ssid: str):
        try:
            subprocess.run(
                [
                    "sudo", "nmcli", "connection", "delete", ssid
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True
            )
        except subprocess.CalledProcessError as e:
            print(f"[WIFI] Error removing Wifi configuration {ssid}: {e.stderr}")

    @staticmethod
    def get_current_ssid():
        try:
            result = subprocess.run(
                ['iwgetid', '-r'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True
            )

            ssid = result.stdout.strip()
            return ssid if (ssid and ssid != "") else None
        except subprocess.CalledProcessError as e:
            print(f"[WIFI] Error getting current SSID: {e.stderr}")

    @staticmethod
    def list_configs():
        try:
            result = subprocess.run(
                ["sudo", "nmcli", "connection", "show"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True
            )
            return result.stdout
        except subprocess.CalledProcessError as e:
            print(f"[WIFI] Error to list profiles: {e.stderr}")


if __name__ == '__main__':
    print(WifiManager.get_current_ssid())
    print(WifiManager.list_configs())
    WifiManager.add_wifi_config("22222", "88888888")
    print(WifiManager.list_configs())
    WifiManager.rm_wifi_config("22222")
    print(WifiManager.list_configs())