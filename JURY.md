# Инструкция для жюри: проверка решения «Резервная одометрия трамвая»

Нода `tram_backup_odometry` (ROS 2 Humble, Python) по трём входным топикам — скоростям передней и задней
тележек и положению ручки контроллера — в реальном времени публикует `/result/velocity` и `/result/position`.
GNSS используется только для начальной выставки в первые секунды записи.

[![Решение в движении: оценка против эталона жюри, режимы фильтра, эпизод проскальзывания](docs/demo.gif)](docs/demo.mp4)

Так решение выглядит во время работы: синяя точка — оценка, кольцо — эталон `/localization/kinematic_state`;
красная полоса — фильтр не верит колёсам (внесённое проскальзывание обеих тележек). Полное видео 2,5 мин —
[`docs/demo.mp4`](docs/demo.mp4); анимация построена по версии из ветки `post-deadline`.

Ниже — три способа проверки: одной командой в Docker (рекомендуется), вручную в ROS 2 и замер задержки.

---

## 0. Что нужно

- ROS 2 Humble (или Docker с образом `ros:humble-ros-base`). Дополнительно — только `numpy`, он есть в образе.
- Запись rosbag2 (`.db3` + `metadata.yaml`).
- Для метрик — пакет судьи `check-code` (`hackathon_solution_checker`) и запись с `/localization/kinematic_state`.

> **Важно про сообщения.** Ноде нужен `tram_vehicle_msgs/DriverControllerCommand` (топик ручки контроллера).
> В пакете `tram_vehicle_msgs` из `check-code` его нет — используйте наш пакет из `ros2_ws/src/tram_vehicle_msgs`.
> `VelocitySensor.msg` в нём идентичен, судья работает без изменений.

---

## 1. Проверка одной командой (Docker, как у судьи)

```bash
cd ros2_ws
docker run --rm \
    -v $PWD:/src_ws:ro \
    -v <путь к check-code>:/check:ro \
    -v <путь к записи>:/bag:ro \
    ros:humble-ros-base bash /src_ws/test_in_docker.sh 1
```

Последний аргумент — скорость воспроизведения (`1` — реальное время).

Скрипт собирает пакеты вместе с судьёй, запускает ноду и судью, проигрывает запись и печатает:

- `=== нода` — начало лога ноды (выставка по GNSS, см. раздел 4);
- `=== судья` — итоговые метрики скорости и положения.

**Ожидаемый результат** на записи организаторов `30618_88aea4d9`:

| | RMSE | 
|---|---|
| скорость | ≈ 0.056 м/с |
| положение, 3D | ≈ 7.2 м |

Прогон в реальном времени занимает столько же, сколько длится запись (≈ 22 мин).

---

## 2. Проверка вручную (ROS 2 Humble)

### Сборка

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select tram_vehicle_msgs tram_backup_odometry
source install/setup.bash
```

### Запуск (три терминала, в каждом `source install/setup.bash`)

```bash
# терминал 1 — нода
ros2 launch tram_backup_odometry odometry.launch.py

# терминал 2 — судья (если нужен расчёт метрик; пакет hackathon_solution_checker собрать в том же workspace)
ros2 run hackathon_solution_checker metrics

# терминал 3 — запись
ros2 bag play <путь к записи>
```

Метрики судья печатает каждые 5 с и итоговые — после остановки (`Ctrl+C` в терминале 2).

---

## 3. Какие топики ожидать

### Входные (из записи)

| Топик | Тип | Единицы |
|---|---|---|
| `/vehicle/front_bogie_velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | км/ч |
| `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | км/ч |
| `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/msg/DriverControllerCommand` | −15…+15 |
| `/sensing/gnss/{master,rover}/fix` | `sensor_msgs/msg/NavSatFix` | только начальная выставка |

### Выходные

| Топик | Тип | Что внутри |
|---|---|---|
| `/result/velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | `velocity` — продольная скорость, м/с |
| `/result/position` | `nav_msgs/msg/Odometry` | `pose.pose.position` — положение base_link в `map`; ориентация — курс пути; `twist.twist.linear.x` — скорость; ковариации заполнены |

- `header.stamp` = время входного сообщения, по которому посчитан результат (время из записи, не стенные часы).
- `header.frame_id` = `map`, `child_frame_id` = `base_link`.
- Частота ≈ 27–30 Гц: выдача на каждую пару сообщений тележек и на каждое сообщение контроллера.
- Пока нет выставки по GNSS, публикуется только скорость (положение ещё неизвестно).

Быстрая проверка:

```bash
ros2 topic list | grep result
ros2 topic hz /result/velocity
ros2 topic hz /result/position
ros2 topic echo /result/position --once
```

---

## 4. Логи ноды

Нода пишет в консоль (при запуске через `test_in_docker.sh` — в `/tmp/node.log`):

| Сообщение | Что значит |
|---|---|
| `данные: …; жду колёса и контроллер` | нода запущена, карта и таблицы загружены |
| `старт: относительная одометрия до выставки по GNSS` | пришли первые сообщения колёс |
| `выставка по GNSS: направление BA, s0 = … м по N RTK-точкам (до линии … м)` | положение привязано к карте маршрута |
| `уточнение по GNSS: …` | уточнение выставки в окне первых 20 с |

Если строки `выставка по GNSS` нет — RTK в начале записи не пришёл, нода работает в режиме относительной
одометрии (`x` — пройденный путь, `y = z = 0`).

Логи судьи — в `/tmp/metrics.log` (Docker) или в терминале 2: строки `Velocity metrics [m/s]` и
`Position metrics [m]` с RMSE, максимумом и числом пар по каждой оси и по 3D-расстоянию.

---

## 5. Задержка и частота

Скрипт `latency_check.py` (корень репозитория) меряет задержку «вход → результат»: время прихода
`/result/velocity` минус время прихода входного сообщения с тем же `header.stamp`.

```bash
# после source install/setup.bash, до запуска ros2 bag play
python3 latency_check.py
```

Каждые 10 с и после `Ctrl+C` печатает медиану, 99-й перцентиль и максимум задержки и частоту обоих выходных топиков.

Требования ТЗ: задержка ≤ 100 мс, частота ≥ 10 Гц. Собственное время обработки одного сообщения в оценщике —
доли миллисекунды; полная задержка включает передачу через DDS.

---

## 6. Параметры

Файл `ros2_ws/src/tram_backup_odometry/config/params.yaml`, подробное описание — в `ros2_ws/README.md`.
Главные для проверки:

| Параметр | Значение | Смысл |
|---|---|---|
| `gnss_window` | 20.0 с | окно RTK-точек для начальной выставки |
| `gnss_wait` | 20.0 с | до выставки положение не публикуется |
| `frame_id` | `map` | система координат положения (как у эталона) |
