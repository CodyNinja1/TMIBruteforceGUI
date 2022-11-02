import sys
from tminterface.interface import TMInterface
import signal
import time
import threading
from bf_gui import *
from bf_tmi import *
from bf_vaf import *

def main():
    server_name = f'TMInterface{sys.argv[1]}' if len(sys.argv) > 1 else 'TMInterface0'
    print(f'Connecting to {server_name}...')
    client = MainClient()
    iface = TMInterface(server_name)

    def handler(signum, frame):
        iface.close()
        quit()

    signal.signal(signal.SIGBREAK, handler)
    signal.signal(signal.SIGINT, handler)
    iface.register(client)

    while not iface.registered:
        time.sleep(0)

    last_finished = False
    last_time = 0
    while iface.registered:
        if last_finished != client.finished:
            last_finished = client.finished
            if last_finished:
                print('Finished')

        if client.time != last_time:
            last_time = client.time
        time.sleep(0)

if __name__ == '__main__':
    x = threading.Thread(target=makeGUI, daemon=True)
    x.start()
    main()
