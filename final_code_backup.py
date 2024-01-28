import socket
import time
from pydub import AudioSegment
from pydub.playback import play
import pyaudio 
import threading
import RPi.GPIO as GPIO
import subprocess

from pynput import keyboard
from pynput.keyboard import Controller
import random

left_ctrl_pressed = False
keyboard_controller = Controller()
server_ip = '0.0.0.0'
server_port = 6000
last_relay_off_time = 0
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
server_socket.bind((server_ip, server_port))
server_socket.listen(3)

FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 512
MAX_PACKET_SIZE = 1024
audio = pyaudio.PyAudio()
reciever_stream = audio.open(format=FORMAT, rate = RATE, output=True, channels=CHANNELS, frames_per_buffer = CHUNK)
sender_stream = audio.open(format=FORMAT, rate = RATE, input=True, channels=CHANNELS, frames_per_buffer=CHUNK)

print(f"Server listening on {server_ip}:{server_port}")
send_audio = True
reception_active = False
send_audio_event = threading.Event()

client_address = None
client_socket = None
send_audio_flag = False
ok = True
# #gpio pin setup
GPIO.setmode(GPIO.BCM)
gpio_pin = 17
gpio_pin2 = 27
gpio_pin3 = 22
gpio_pin4 = 23
GPIO.setup(gpio_pin, GPIO.OUT)
GPIO.setup(gpio_pin2, GPIO.OUT)
GPIO.setup(gpio_pin3, GPIO.OUT)
GPIO.setup(gpio_pin4, GPIO.OUT)
GPIO.output(gpio_pin, GPIO.HIGH)#RELAY
GPIO.output(gpio_pin2, GPIO.LOW)#YELLOW LED
GPIO.output(gpio_pin3, GPIO.LOW)#GREEN LED
GPIO.output(gpio_pin4, GPIO.HIGH)#RED LED

numberOfConnection = 0

timeout_duration = 1.25
last_data_time = time.time()

relay_status = 0
receiving_audio = False  # Flag to indicate if audio reception is active

def check_timeout_and_turn_off_relay():
    global last_data_time, relay_status, ok
    while True:
        # Calculate the time elapsed since the last data was received
        time_elapsed = time.time() - last_data_time
        # If no new data received for more than the timeout duration, turn off the relay
        if time_elapsed >= timeout_duration:
            relay_status = 0 #matlb relay band hogi 
            GPIO.output(gpio_pin, GPIO.HIGH) #realy band karti pra
            GPIO.output(gpio_pin2, GPIO.LOW) #realy band karti pra
            GPIO.output(gpio_pin3, GPIO.LOW) #realy band karti pra
            GPIO.output(gpio_pin4, GPIO.HIGH) #realy band karti pra
            print('relay turned off')
            ok=True
            
        # Sleep for a short interval before checking again
        time.sleep(0.1)

def on_key_release(key):
    global left_ctrl_pressed, relay_status, reception_active

    if key == keyboard.Key.ctrl_l:
        left_ctrl_pressed = False
        if relay_status == 1:
            reception_active = False  # Reception is now complete
        print('key released')
        
def on_key_press(key):
    global left_ctrl_pressed, relay_status
    if relay_status==0 or key == keyboard.Key.ctrl_l:
        left_ctrl_pressed = True
        print('key pressed')

def check_keypresses():
    with keyboard.Listener(on_release= on_key_release, on_press = on_key_press) as listener:
        listener.join()

def run_python_file(file_name):
    while True:
        try:
            process = subprocess.Popen(["python", file_name])
            time.sleep(500)  # Wait for 2 seconds
        except KeyboardInterrupt:
            print("Terminating the program.")
            break

def send_audio():
    global relay_status, client_socket, left_ctrl_pressed, send_audio_flag, reception_active, receiving_audio , ok

    while True:
        if True:
            try:
                while True:
                    data = sender_stream.read(CHUNK)
                    if ok and client_socket is not None:
                        client_socket.send(data)
                        print("Sending audio:", len(data), "bytes")
                        GPIO.output(gpio_pin4, GPIO.LOW) #realy band karti pra
                        GPIO.output(gpio_pin3, GPIO.HIGH) #realy band karti pra

                        
            except Exception as e:
                print(f"Error sending audio ew: {type(e)}")
                python_file = "/home/army/Desktop/pythonserver.py"  # Replace this with your Python file name
                run_python_file(python_file)
        else:
            time.sleep(0.1)
            
            
            
def recieve_audio():
    while True:
        try:
            global client_address, relay_status, last_data_time, client_socket, last_relay_off_time, send_audio_flag, send_audio, receiving_audio , ok 
            if (numberOfConnection < 4):
                client_socket, client_address = server_socket.accept()
                print(f"Accepted connection from {client_address}")
                send_audio_flag = True  # Set the flag to start sending audio after connection
                send_audio = True  # Start sending audio immediately
                receiving_audio = True  # Indicate that audio reception is active
        except Exception as e:
            print(f'connection closed :{e}')
            python_file = "/home/army/Desktop/pythonserver.py"  # Replace this with your Python file name
            run_python_file(python_file)
            

        try:
            while True:
                data = client_socket.recv(4096)
                data = data.strip()
                print(len(data), 'lala')
                if len(data) == 4:
                    print("\nrelay on", len(data))
                    if time.time() - last_relay_off_time >= timeout_duration:
                        relay_status = False
                        send_audio_event.clear()  # Pause sending audio
                        receiving_audio = True  # Indicate that audio reception is active
                        ok = False
                        GPIO.output(gpio_pin, GPIO.LOW)
                        GPIO.output(gpio_pin2, GPIO.HIGH)
                        GPIO.output(gpio_pin3, GPIO.LOW)

                if len(data) == 3:
                    print("\nrelay off", type(data))
                    GPIO.output(gpio_pin, GPIO.HIGH)
                    GPIO.output(gpio_pin2, GPIO.LOW)
                    relay_status = True
                    last_relay_off_time = time.time()
                    send_audio_event.set()  # Resume sending audio
                    receiving_audio = False  # Indicate that audio reception is complete
                    ok = True
                    
                if not data:
                    break

                if True:
                    reciever_stream.write(data)
                    last_data_time = time.time()

        except Exception as e:
            client_socket.close()
            print(f"Error: {e}")
            GPIO.cleanup()

        finally:
            GPIO.cleanup()
            client_socket.close()
            server_socket.close()
            print("Connection closed")
def record_and_send_audio():
    global relay_status, client_socket, send_audio_flag, send_audio

    while True:
        if send_audio:
            try:
                while sender_stream.get_read_available() >= CHUNK:
                    sender_stream.read(CHUNK)

                while send_audio_flag:
                    data = sender_stream.read(CHUNK)
                    if data:
                        client_socket.send(data)
                        print("Sending audio:", len(data), "bytes")
            except Exception as e:
                print(f"Error sending audio: {e}")

        else:
            time.sleep(0.1)
send_audio_thread = threading.Thread(target=send_audio)
recieve_audio_thread = threading.Thread(target=recieve_audio)
timeout_thread = threading.Thread(target=check_timeout_and_turn_off_relay)
check_thread = threading.Thread(target=check_keypresses)

send_audio_thread.start()
recieve_audio_thread.start()
timeout_thread.start()
check_thread.start()
