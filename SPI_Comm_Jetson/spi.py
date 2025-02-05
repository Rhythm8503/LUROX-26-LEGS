import spidev
import time

# SPI configuration
SPI_BUS = 0  # Change if needed based on your Jetson SPI interface
SPI_DEVICE = 0  # Change based on the connected SPI device
BUFFER_SIZE = 8  # Match ESP32 buffer size

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = 1000000  # Adjust speed as needed
spi.mode = 0  # Must match ESP32's SPI_MODE0

def send_spi_command(command):
    """ Send a single byte command to the ESP32 """
    tx_buf = [command] + [0] * (BUFFER_SIZE - 1)  # Pad buffer to match ESP32 size
    rx_buf = spi.xfer2(tx_buf)  # Perform SPI transaction
    print(f"Sent: {command}, Received: {rx_buf}")

try:
    while True:
        user_input = input("Enter a number (0-255) to send via SPI: ")
        if user_input.isdigit():
            send_spi_command(int(user_input))  # Convert input to integer and send
        else:
            print("Invalid input. Please enter a number between 0 and 255.")
        time.sleep(0.5)  # Delay for stability

except KeyboardInterrupt:
    print("Closing SPI connection.")
    spi.close()
