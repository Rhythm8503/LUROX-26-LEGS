import spidev
import time

# Initialize SPI connection (assuming bus 0, device 0)
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0
spi.max_speed_hz = 50000  # Set SPI clock speed (adjust as needed)
spi.mode = 0b00  # SPI Mode 0 (CPOL=0, CPHA=0)

# Send 1 byte of data (e.g., 0x42)
data = [0x42]  # 1 byte of data to send
spi.xfer2(data)  # Transfer the data

time.sleep(1)  # Wait for a while before closing
spi.close()  # Close SPI connection
