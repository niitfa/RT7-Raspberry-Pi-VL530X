Инструкция по запуску датчика

1. Установить wiringOP
# apt-get update
# apt-get install -y git
# git clone https://github.com/orangepi-xunlong/wiringOP.git
# cd wiringOP
# ./build clean
# ./build 

2.  Настроить интерфейс i2c (i2c2-m0) (Physical Pins: 3 (SDA.2) и 5 (SCL.2))
Открыть файл конфигурации
# sudo nano /boot/orangepiEnv.txt
В конец файла добавить строку
overlays = i2c2-m0
Перезагрузить компьютер
# reboot

2. Собрать библиотеку libDistSensor.so для датчика дыхания из этого проекта
Клонировать  репозиторий. Из папки проекта
# cmake .
# make

Запустить пример для проверки
./run

Библиотеку libDistSensor.so использовать в проекте с графичеким интерфейсом.

