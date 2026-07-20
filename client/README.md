# rfid — C++ CLI для UHF RFID-считывателя

Утилита командной строки для считывателя по протоколу «Unify Standard». Порт `../reader/*.py` на C++17: тот же формат кадров, плюс полный набор команд из
`documantation/sdk/.../Document/Protocol/Protocol_20250419-En.pdf`.

## Сборка (CMake)

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
# бинарь: client/build/rfid
```

Зависимостей нет — только стандартная библиотека и POSIX-сокеты/termios (Linux/macOS). Один файл без CMake тоже соберётся:
```bash
g++ -std=c++17 -O2 -o rfid main.cpp protocol.cpp transport.cpp
```

Тесты (CTest) — проверяют кодек кадров и парсер на примерах из PDF-протокола, устройство не нужно:
```bash
cd build && ctest --output-on-failure
```

## Подключение

Ровно один из флагов:
```bash
rfid --tcp 192.168.1.200:6000  <команда> [аргументы]
rfid --serial /dev/ttyUSB0:115200 <команда> [аргументы]
```
Baud по умолчанию 115200. Общие опции: `--device N`, `--epc HEX`, `--pwd HEX` (4 байта),
`--tid-match`, `--nosave`, `--timeout SEC` (ожидание ответа на команду, по умолчанию 3с —
команды, которые прошивка игнорирует, завершаются с ошибкой, а не виснут).

## Команды

| Команда | CMD | Описание |
|---|---|---|
| `version` | 0x01 | версия hw/sw |
| `info` | 0x07 | дамп информации об устройстве |
| `inventory` | 0x02/0x03 | непрерывное чтение, Ctrl+C — стоп |
| `single` | 0x8E | один проход по каждой антенне |
| `get-area` / `set-area <0..6>` | 0x81/0x80 | банк памяти инвентаризации |
| `read <reserved\|epc\|tid\|user> <addr> <words>` | 0x83 | чтение памяти метки |
| `write <reserved\|epc\|tid\|user> <addr> <dataHEX>` | 0x82 | запись памяти метки |
| `lock <kill\|access\|epc\|tid\|user> <open\|permaopen\|lock\|permalock>` | 0x8A | блокировка |
| `kill` | 0x8B | деактивация метки (нужен `--pwd`) |
| `get-freq` / `set-freq <band> <MHz> <kHz> <stepkHz> <count>` | 0x85/0x84 | частота |
| `get-antenna` / `set-antenna <mask-hex> <ant:pwr:onMs:gapMs>…` | 0x87/0x86 | антенны |
| `detect-antenna` | 0x8C | какие антенны подключены |
| `restart` / `factory-reset` | 0x04/0x05 | перезагрузка / сброс |
| `raw <cmd-hex> [body-hex]` | — | произвольная команда |

## Примеры

```bash
rfid --tcp 192.168.1.200:6000 inventory
rfid --serial /dev/ttyUSB0 version
# записать AABBCCDD в User-память со смещения 2 (в словах), пароль 12345678
rfid --tcp 192.168.1.200:6000 write user 2 AABBCCDD --pwd 12345678
# три частоты 920.5/921.0/921.5 МГц, band GB-2
rfid --tcp 192.168.1.200:6000 set-freq 0 920 500 500 3
# антенны 1 и 4 вкл, 30 dBm, 200мс работа / 100мс пауза
rfid --tcp 192.168.1.200:6000 set-antenna 09 1:30:200:100 2:30:200:100 3:30:200:100 4:30:200:100
```

## Устройство кода

- `protocol.hpp/.cpp` — формат кадра (`build_frame`/`read_frame`), разбор отчётов о метках (`parse_tag_report`, включая multi-area класс `0x0D`), `build_match_prefix`. Проверено на примерах кадров из PDF-протокола.
- `transport.hpp/.cpp` — `TcpTransport` (сокеты, connect с таймаутом) и `SerialTransport` (termios, 8N1). Оба дают `open/close/write/read_exact`; сигнал (Ctrl+C) прерывает блокирующее чтение через `Interrupted`.
- `main.cpp` — разбор аргументов, класс `Client` (request/response с пропуском асинхронных heartbeat и tag-report кадров) и реализации команд.

## Замечание про документацию

В PDF-протоколе пример «простого» кадра метки (раздел 3.2) содержит опечатку в поле длины (`0x11` вместо `0x13`) — тело там 17 байт, что подтверждается контрольной суммой. Формула `body = LEN − 2` (как в `reader/protocol.py` и здесь) корректна и согласуется с остальными примерами (multi-area, version).
