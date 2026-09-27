# Резервная одометрия трамвая — ROS 2 Humble

Нода `tram_backup_odometry` в реальном времени оценивает продольную скорость и положение трамвая **только по трём
топикам** — скоростям передней и задней тележек и положению ручки контроллера — и публикует `/result/velocity`
и `/result/position`. GNSS не обязателен: если есть RTK в первые секунды — только начальная выставка (окно 20 с);
если нет — относительная одометрия.

## Состав

```
ros2_ws/src/
├── tram_vehicle_msgs/            сообщения VelocitySensor и DriverControllerCommand
│                                 (пакет из датасета + DriverControllerCommand.msg; VelocitySensor.msg совпадает с пакетом жюри)
└── tram_backup_odometry/         нода (Python, зависимости: rclpy, nav_msgs, sensor_msgs, numpy)
    ├── tram_backup_odometry/node.py        ROS-обвязка: подписки, пары тележек, выставка по GNSS, публикация
    ├── tram_backup_odometry/core/          оценщик (IMM + модель привода + карта), выставка, карта
    ├── data/                               карта маршрута №10 и откалиброванные таблицы (офлайн)
    ├── config/params.yaml                  параметры
    └── launch/odometry.launch.py
```

> Если в вашем workspace уже есть `tram_vehicle_msgs` **только с `VelocitySensor`** (как в `check-code`), замените его
> нашим: ноде нужен ещё `DriverControllerCommand` для топика `/vehicle/driver_position_cmd`. `VelocitySensor.msg`
> идентичен, поэтому судья работает с ним без изменений.

## Сборка

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select tram_vehicle_msgs tram_backup_odometry
source install/setup.bash
```

Дополнительных пакетов ставить не нужно: только `numpy` (есть в `ros:humble-ros-base`).

## Запуск

```bash
ros2 launch tram_backup_odometry odometry.launch.py      # нода с параметрами из config/params.yaml
ros2 bag play <bag>                                      # в другом терминале
```

Или без launch-файла: `ros2 run tram_backup_odometry odometry`.

## Контракт

| | топик | тип | значение |
|---|---|---|---|
| вход | `/vehicle/front_bogie_velocity` | `tram_vehicle_msgs/VelocitySensor` | км/ч (как в данных) |
| вход | `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/VelocitySensor` | км/ч |
| вход | `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/DriverControllerCommand` | −15…15 |
| выставка | `/sensing/gnss/{master,rover}/fix` | `sensor_msgs/NavSatFix` | только RTK-точки первых 20 с |
| выход | `/result/velocity` | `tram_vehicle_msgs/VelocitySensor` | `velocity`, м/с |
| выход | `/result/position` | `nav_msgs/Odometry` | `pose.pose.position` в `map` (система Pathgraph, как у эталона) |

- `header.stamp` выходных сообщений = `header.stamp` входного сообщения, на котором посчитан результат.
- Выдача на каждое сообщение колёс (пара передняя/задняя с одним stamp) и на каждое сообщение контроллера
  (прогноз по модели привода, состояние фильтра не меняется): ≈ 27–30 Гц.
- Время обработки сообщения — доли миллисекунды (требование ≤ 100 мс).
- В `/result/position` также заполнены ориентация (курс пути по карте), скорость в `twist` и ковариации:
  вдоль пути и скорости — из фильтра (откалиброваны по данным), поперёк пути и высоты — грубые константы из YAML.
- Пока нет выставки, `/result/position` не публикуется (скорость — публикуется): нода ждёт RTK до `gnss_wait` = 5 с,
  чтобы не выдать положение «около нуля» вместо координат карты.
- Без GNSS на старте (или если он дальше 30 м от маршрута) — относительная одометрия: `x` — пройденный путь, `y = z = 0`.
  Если RTK придёт позже — выставка по нему, положение переходит в систему карты.

## Проверка судьёй жюри (Docker)

```bash
# из каталога ros2_ws; check-code — пакет судьи жюри, <bag> — запись с /localization/kinematic_state
docker run --rm -v $PWD:/src_ws:ro -v <check-code>:/check:ro -v <bag>:/bag:ro \
    ros:humble-ros-base bash /src_ws/test_in_docker.sh 1      # 1 — скорость воспроизведения
```

Скрипт собирает пакеты вместе с `hackathon_solution_checker`, запускает ноду и судью, проигрывает запись и печатает
последний отчёт судьи.

## Параметры (`config/params.yaml`)

| параметр | по умолчанию | смысл |
|---|---|---|
| `gnss_window` | 20.0 | с — окно RTK-точек для начальной выставки |
| `gnss_max_dist` | 30.0 | м — дальше от обеих линий: вне карты → относительная одометрия |
| `gnss_wait` | 5.0 | с — сколько ждать RTK до перехода в относительную одометрию (положение до этого не публикуется) |
| `publish_on_cmd` | true | выдавать и на сообщения контроллера |
| `stop_exit` | 0.5 | с — обе тележки ровно 0 столько подряд → стоянка (выход из режима «обе проскальзывают») |
| `slip_rollback` | true | неподтверждённое проскальзывание обеих → путь за эпизод пересчитывается по колёсам |
| `slip_no_neutral` | true | при ручке в нейтрали (u = 0) в режим «обе проскальзывают» не входить: торможение бывает тормозом вне топика |
| `sigma_v_scale`, `kappa_sigma` | 1.534, 0.0062 | калибровка неуверенности (95 % ошибок в ±2σ на обучающих рейсах) |
| `sigma_cross`, `sigma_z` | 1.0, 0.5 | м — поперечная и вертикальная неуверенность в covariance (не калибровались) |
| `pair_tol`, `pair_wait` | 0.005, 0.05 | с — сведение сообщений передней и задней тележки в пару |
