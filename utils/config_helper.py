import os
import configparser

def update_config_ip(ip, filename="config.ini"):
    config = configparser.ConfigParser()

    if not os.path.exists(filename):
        create_config_file(filename)
    config.read(filename)
    config['controller_network']['ip'] = ip
    with open(filename, 'w') as configfile:
        config.write(configfile)
    print(f"Updated IP in config file to: {ip}")

def update_config_port(port, filename="config.ini"):
    config = configparser.ConfigParser()

    if not os.path.exists(filename):
        create_config_file(filename)
    config.read(filename)
    config['controller_network']['port'] = str(port)
    with open(filename, 'w') as configfile:
        config.write(configfile)
    print(f"Updated Port in config file to: {port}")


def create_config_file(filename="config.ini"):
    config = configparser.ConfigParser()

    config['controller_network'] = {
        'ip': '',
        'port': ''
    }

    with open(filename, 'w') as configfile:
        config.write(configfile)
    print(f"Created Config file '{filename}'")