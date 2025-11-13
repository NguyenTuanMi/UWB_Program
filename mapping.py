#from UWB_ReadUDP import get_target_position, get_all_positions
import time
from pynput import keyboard

N = int(input("Number of obstacles: "))
L = []
LL = []
def on_press(key, injected):
    global N, L
    if key.char != 'a':
        a, b = [], []
        for _ in range(5):
            #pos = get_target_position(1, 15)
            pos = [1, 2]
            a.append(pos[0])
            b.append(pos[1])
        L.append((sum(a)*100/len(a), sum(b)*100/len(b)))
    else:
        if N == 0:
            print(L)
            L = []
        else:
            LL.append(L)
            print(LL)
            L = []
            N = N-1
def on_release(*args):
    pass
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()
time.sleep(1000)
