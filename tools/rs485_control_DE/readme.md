## Usage
### Install Required Libraries
``` bash
sudo apt-get install libgpiod-dev
```

### Compilation
``` bash
gcc -o rs485_DE rs485_DE.c -lgpiod
```
### Execution
``` bash
sudo ./rs485_DE TTY_DIR DE_CHIP DE_LINE [rs485_dir] [EN_CHIP] [EN_LINE]
```
### Parameter Explanation
TTY_DIR: Serial port device name, e.g., /dev/ttyAMA0
DE_CHIP: gpiochip device number for the DE pin chip, e.g., /dev/gpiochip0 
DE_LINE: Number of the DE pin, e.g., 0 
rs485_dir: The directory of the created device file, e.g., "/dev/ttyAMA10"
EN_CHIP: gpiochip device number for the enable control pin chip (optional), e.g., /dev/gpiochip0 
EN_LINE: Number of the enable control pin (optional), e.g., 0

## Results

A device file in [rs485_dir] will be created. By utilizing this device file, automatic switching between RS485 transmission and reception states can be effortlessly accomplished.