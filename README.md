# K1948BK018_ASM_OUT
Управление выводом (GPIO) первого 100% отечественного микроконтроллера из ассемблерного участка кода (в проекте присутствует также ОС FreeRTOS на два потока). | О подключении FreeRTOS можно прочитать здесь: https://wiki.mik32.ru/%D0%97%D0%B0%D0%BF%D1%83%D1%81%D0%BA_FreeRTOS_%D0%BD%D0%B0_MIK32 | Этот проект сделан на основе среды разработки: https://files.mik32.ru/eclipse/mcu32-ide-v0-2-2.zip (у меня версия v0.0.6, ныне устаревшая, но там был рабочий пример для FreeRTOS) | Скрипт elbear_uploader.py для загрузки прошивки в ПЗУ скачан по ссылке: https://gitflic.ru/project/elron-tech/elbear_uploader Файл main.c содержит две ассемблерные (32-битная архитектура RISC-V) вставки в двух разных потоках ОС. Одна из них приводит к формированию пачек из 12 импульсов на выводе ASM_OUT (определён в единственном месте кода). Другая - воздействует на группу регистров ядра, которые используются в первой. Подобными ASM-вставками можно ускорять работу ПРОГРАММНЫХ SPI/I2C/UART/1W/… (не надеясь на автоматическую оптимизацию компилятора), если на данных выводах отсутствуют аппаратные модули необходимых интерфейсов.
