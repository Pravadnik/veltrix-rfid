# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Язык

Отвечай на русском, кратко. База данных чипов и вся документация ведутся на русском.

## Что это за проект

Три независимые, но связанные части вокруг UHF RFID:

1. `reader/` — Python-библиотека протокола RFID-считывателя ("Unify Standard").
2. `client/` — C++17 CLI-утилита `rfid` для того же считывателя (порт `reader/` + полный набор команд).
3. `database/` — датащиты чипов (Impinj, NXP, Alien) в Markdown по единой методике верификации.
4. `web/` — статический одностраничный просмотрщик базы чипов, генерируемый из `database/`.

`documantation/` — PDF-протоколы, руководства и вендорский Java-SDK (`Reader Comprehensive DEMO_Java_SDK_V4.10`). Первоисточник формата кадров — `documantation/sdk/.../Document/Protocol/Protocol_20250419-En.pdf`.

## Команды

Пересборка данных для веб-интерфейса (обязательно после правок в `database/**/*.md`):
```bash
python3 web/build.py     # парсит database/ → web/data.js
```

Просмотр веб-интерфейса — открыть `web/index.html` в браузере (сборки нет; `data.js` — единственный сгенерированный артефакт).

Библиотека `reader/` не имеет CLI/точки входа — это `protocol.py` + `transport.py`, импортируемые как модули. Для serial-транспорта нужен `pyserial`; TCP работает на стандартной библиотеке.

Сборка C++ CLI (`client/`):
```bash
cmake -S client -B client/build -DCMAKE_BUILD_TYPE=Release && cmake --build client/build
# бинарь: client/build/rfid ; см. client/README.md
(cd client/build && ctest --output-on-failure)   # юнит-тесты кодека кадров
```

## Архитектура: reader/

Реализация бинарного протокола считывателя, разделённая на два слоя:

- **`transport.py`** — байтовый слой. `TcpTransport` и `SerialTransport` дают одинаковый минимальный интерфейс: `open/close/write(bytes)/read_exact(n)`. `read_exact` блокирует до получения ровно `n` байт либо кидает `TransportClosed`. `parse_endpoint()` разбирает `host:port` (tcp) и `PORT[:BAUD]` (serial).
- **`protocol.py`** — формат кадра и разбор отчётов о метках. Кадр: `43 4D | CMD | EIG | LEN(2,LE) | DEVICE_NO(2,LE) | BODY | CHECKSUM(XOR)`. `build_frame()` собирает host→device кадр, `read_frame(read_exact)` читает device→host. Ключевая тонкость: **CMD `0x02` перегружен** — тело в 1 байт это ack старта/стопа инвентаризации, тело ≥3 байт это отчёт о метке; различает их `is_tag_report_body()`, разбирает `parse_tag_report()`. Опциональные поля отчёта (антенна, RSSI, PC, timestamp, user data) включаются битами `options` в фиксированном порядке. Для класса `0x0D` (UHF multi-area) тело метки дополнительно разбирается `parse_multi_area()` на блоки Reserved/EPC/TID/User.

При добавлении команд протокола расширяй enum `Cmd` и, если это read/write/lock/kill (0x82/0x83/0x8A/0x8B), переиспользуй общий префикс `build_match_prefix(match_epc, password)`.

## Архитектура: database/ + web/

Однонаправленный конвейер: Markdown-датащиты — источник истины, `web/build.py` их компилирует, фронтенд их только читает.

- **`database/<vendor>/<chip>.md`** — по файлу на чип (`impinj/`, `nxp/`, `alien/`). Строгая структура, от которой зависит парсер: заголовок `# Название`, строка верификации `> ...`, и таблицы под заголовками **`Technical Summary`** (столбцы `Параметр | Значение | Статус`), **`Feature Table`** (`Категория | Параметр | Значение | Статус`). Каждое поле помечается `Confirmed` / `Estimated` / `Unknown` — это методология верификации, не украшение (см. `database/impinj/README.md`). Связи между чипами — вики-ссылками `[[slug]]`; чип без своей Feature Table может наследовать данные через `[[...]]` (поле `inherits_from`).
- **`README.md`** каждого вендора содержит карту `TID Model Number → чип` и общую сравнительную таблицу; `build.py` берёт статус чипа из impinj README, если он там переопределён.
- **`web/build.py`** — регексами вытаскивает TID-префикс, биты EPC/User, пароли, спец-фичи (QT/Crypto/FastID/…) и счётчики статусов, пишет `window.CHIP_DATA` в `web/data.js`. Парсер завязан на **русские** заголовки столбцов (`параметр`, `значение`, `статус`, `чип`) — сохраняй их при правке датащитов, иначе поля не подхватятся.
- **`web/app.js`** — ванильный JS (без сборщика), читает `data.js`, рендерит карточки/таблицу, фильтры и детальный вид (Markdown через `web/lib/marked.min.js`). `web/data.js` **генерируемый** — правь `.md` и пересобирай, а не его.

## Замечание про git-историю

История ветки содержит коммиты про ESP/печать чеков от другого проекта в этом репозитории — к RFID-коду выше отношения не имеют. Ориентируйся на текущее содержимое каталогов, а не на прошлые commit-сообщения.
