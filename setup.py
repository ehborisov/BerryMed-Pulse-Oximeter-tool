from setuptools import setup

setup_config = {
    'description': 'Python script to connect and retrieve data from Bluetooth Pulse Oximeter BerryMed BM1000C.',
    'author': 'Borisov Egor',
    'version': '1.0',
    'packages': ['berrymed_pulse_oximeter'],
    'install_requires': [
        'Adafruit-BluefruitLE>=0.9.10'
        'construct>=2.9.45'
    ],
    'name': 'berrymed_pulse_oximeter'
}


def main():
    setup(**setup_config)


if __name__ == '__main__':
    main()
