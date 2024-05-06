# BE LoRa

## arduino-cli

give rights to interact with the arduino
```sh
sudo chmod a+wr /dev/ttyACM0
```

select the board
```sh
arduino-cli board attach -p /dev/ttyACM0 main.ino -b arduino:avr:mega
```

compile the code
```sh
arduino-cli compile main.ino
```

upload the code
```sh
arduino-cli upload main.ino
```

monitor the responses and log them
```sh
echo "" > logs.txt
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200 | tee logs.txt
```


