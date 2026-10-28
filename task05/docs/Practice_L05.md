# Практика L05 Управление суставами

Ориентир — 90 минут, с дополнительным временем на вопросы и первые команды. Преподаватель выполняет шаг, студенты повторяют его. Рабочая практика уже настроена. Основной результат: neutral/standing posture → небольшой wave → neutral → измеренная остановка при отмене. Таз закреплён; это упражнение на управление суставами.

## 0–10 минут Подготовка

Выполняйте команды из `task05/`. Первая сборка образа должна быть закончена до занятия.

```bash
./course up
./course build
./course doctor
./course desktop
```

Откройте ссылку рабочего стола в браузере. Для следующих частей используйте два терминала из этой папки; для записи одновременно с движением понадобится третий, а для отмены во время записи — четвёртый.

Checkpoint: окружение доступно, пакет `control_lab` установлен. Если Docker не работает, приложите вывод `docker compose ps` и `./course doctor`, прежде чем менять robot/controller config.

## 10–25 минут Один сустав и обратная связь

Сначала найдите на рисунке или модели локоть, привод и измеряемый угол. Назовите желаемое положение и положение, которое сустав занимает сейчас. Пример для локтя: желаемое −0.60 rad, измеренное −0.45 rad; ошибка −0.15 rad. Прежде чем вводить gain, объясните, какое движение уменьшит эту ошибку. Радианы описывают угол; N·m описывают момент привода.

В этом опыте запускается математическая модель одного сустава. Это calculated experiment, отдельный от Gazebo. Модель, units и параметры записываются в JSON; графики не являются измерениями гуманоидной модели.

```bash
./course pid --profile p --output /workspace/evidence/pid-p.json --plot /workspace/evidence/pid-p.png
```

Откройте P plot, найдите reference, measurement и effort. Большая ошибка вызывает более сильную коррекцию. После этого, по готовности группы, выполните дополнительное сравнение:

```bash
./course pid --profile pd --output /workspace/evidence/pid-pd.json --plot /workspace/evidence/pid-pd.png
./course pid --profile pid --output /workspace/evidence/pid-pid.json --plot /workspace/evidence/pid-pid.png
```

Откройте PNG из `evidence/` локально. На PD plot движение затормаживается около цели. Для PID накопленная ошибка помогает компенсировать постоянную нагрузку. При недостатке времени достаточно графика P и качественного сравнения; подробный PID-разбор — дополнительный опыт. Используйте фактические параметры из JSON: одинаковое слово PID не задаёт универсальные gains или форму отклика.

Checkpoint: студент объясняет разницу между положением сустава и actuator effort, а также роль обратной связи. Увеличение gain требует оценки отклика, шага и пределов.

## 25–40 минут Модель и controllers

В первом терминале:

```bash
./course practice
```

Оставьте этот запуск работающим. Для CPU/headless режима вместо него используйте `./course practice headless`; измерения и команды остаются теми же.

Во втором терминале:

```bash
./course control list
./course control interfaces
```

Найдите `joint_state_broadcaster` и `humanoid_controller`. Проверьте active state и 18 position command interfaces. State interfaces содержат position/velocity. По описанию модели сопоставьте четыре сустава каждой руки и пять суставов каждой ноги.

Модель имеет закреплённый таз. Ноги не поддерживают баланс свободного тела. Resource Manager хранит hardware components и interfaces; Controller Manager управляет lifecycle controllers и их update. `robot_state_publisher` строит TF по joint states; Gazebo исполняет динамику и simulator servo. У этих компонентов разные обязанности.

Повторите основные ROS 2 команды без `course`-обёртки. В новом **терминале host**, из `task05/`:

```bash
docker compose exec workspace bash
```

Теперь вы внутри **container shell**. Выполните sourcing в этом shell:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 node list
ros2 topic list -t
ros2 topic echo /humanoid/joint_states --once
ros2 control list_controllers -c /humanoid/controller_manager
ros2 control list_hardware_interfaces -c /humanoid/controller_manager
ros2 action list -t
```

Каждому measured position сопоставьте его joint name. Gazebo play/pause управляет продвижением simulated time; camera controls меняют вид сцены. RViz показывает ROS model/TF и имеет отдельную камеру. После resume simulation time продолжает идти. Перед отправкой trajectory убедитесь, что физика запущена. Закрытие GUI отдельно от server не завершает весь launch.

Checkpoint: объясните цепочку trajectory → controller → command interface → simulator → measured joint state. Если controller inactive, сначала смотрите доступность interfaces, корректность config и logs.

## 40–55 минут Первая траектория

Во втором терминале:

```bash
./course control pose neutral --duration 3
```

Дождитесь окончания движения и удержания neutral, затем отправьте следующую команду:

```bash
./course control pose reach --duration 3
```

Эти готовые учебные команды публикуют trajectory через стандартный ROS CLI. Topic-команда возвращается после публикации и не ждёт завершения движения: следующая команда может заменить ещё исполняемую trajectory. Во время движения следите за руками. Сравните задаваемые суставы с измеренными; визуальное движение само по себе не доказывает правильность порядка joint names.

Сначала верните модель в neutral и дождитесь action result:

```bash
./course control goal neutral --duration 3
```

Для записи отклика запустите во втором терминале:

```bash
./course measure --duration 12 --output /workspace/evidence/practice.json
```

В третьем терминале во время записи:

```bash
./course control pose reach --duration 3
```

После завершения записи:

```bash
./course report --input /workspace/evidence/practice.json
./course plot --input /workspace/evidence/practice.json --output /workspace/evidence/practice.png
```

По JSON и графику назовите commanded/desired state, measured state, время и units. Различайте desired trajectory state и конечную позу: во время интерполяции конечная цель ещё не является текущим reference.

Checkpoint: объясните tracking error во время движения и после окончания. Все времена траектории задаются относительно её начала; переход к другой конечной позе требует нового набора point values.

## 55–70 минут Trajectory action

```bash
./course control goal neutral --duration 3
./course control goal reach --duration 8
```

Команда использует стандартный `ros2 action send_goal` с feedback. Разберите отдельно принятие цели, feedback и окончательный result. Принятие цели означает согласие controller её исполнять; успех определяется выполнением action contract и tolerances.

Перед повторным запуском измените длительность одной команды на 5 секунд и предскажите различие скорости. Не меняйте gains без объяснения: position command в этом комплекте исполняет simulator servo, а JTC формирует согласованную trajectory.

Теперь выполните полный **stand/neutral → wave → neutral** через ROS CLI. Откройте [static goal example](../examples/practice_wave_goal.yaml): он содержит шесть points и все 18 joints. Для каждого point positions идут в порядке joint_names; десять суставов ног остаются в neutral. Этот пример предназначен для совместного разбора команд и измерений.

Подготовьте оба sourced container shells и команды до начала записи. В одном shell сначала запустите recorder:

```bash
ros2 run control_lab lab measure --duration 30 --timeout 180 --output /workspace/evidence/wave.json
```

Пока запись идёт, в другом sourced **container shell**:

```bash
ros2 action send_goal \
  /humanoid/humanoid_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  --stdin --feedback < /workspace/examples/practice_wave_goal.yaml
```

`--duration 30` задаёт simulated seconds записи; `--timeout 180` ограничивает ожидание в wall seconds. На медленной машине эти часы продвигаются с разной скоростью.

После завершения записи:

```bash
ros2 run control_lab lab plot --input /workspace/evidence/wave.json --output /workspace/evidence/wave.png --joints left_shoulder_roll left_elbow_pitch left_wrist_roll
```

Найдите подъём руки, смену направления wrist и возврат в neutral. `--stdin` читает YAML message из файла; goal принятие, feedback и result относятся к одному action. В примере времена 1, 3, 4, 5, 6 и 9 s отсчитываются от начала trajectory. Полный goal занимает 9 simulated seconds.

Checkpoint: найдите joint names, позиции и `time_from_start` в goal; объясните, почему просто переставить имена при прежнем порядке positions нельзя.

## 70–82 минут Отмена и удержание

Сначала верните модель в reach и дождитесь завершения action, чтобы отмена происходила во время реального движения:

```bash
./course control goal reach --duration 3
```

В третьем терминале включите новую запись состояния:

```bash
./course measure --duration 18 --output /workspace/evidence/cancel.json
```

Во втором терминале запустите длительную цель из позы reach:

```bash
./course control goal neutral --duration 12
```

Через 2–3 секунды, пока движение продолжается, в четвёртом терминале:

```bash
./course control cancel
```

После окончания записи постройте график:

```bash
./course report --input /workspace/evidence/cancel.json
./course plot --input /workspace/evidence/cancel.json --output /workspace/evidence/cancel.png
```

Смотрите на result и измеренное движение до и после отмены. Cancel response и физическая остановка — отдельные наблюдения. Положение удержания может зависеть от выбранной controller policy; нулевая torque не является универсальной командой удержания. Для подтверждения удержания сравните measured position до отмены и после неё; не считайте один cancel response доказательством остановки.

После обсуждения верните модель в neutral:

```bash
./course control pose neutral --duration 3
```

Checkpoint: студент называет наблюдения, которыми подтверждается остановка, и объясняет, почему поведение при отмене нужно задать явно.

## 82–90 минут Повторение и завершение

По модели и измерениям проследите весь путь: trajectory goal → controller → command interface → simulator → state interface → измеренное положение. Назовите один пример различия reference и measurement, поясните роль joint names и cumulative `time_from_start`, а затем перечислите наблюдения, которыми подтверждается остановка при отмене.

В первом terminal нажмите Ctrl+C. При необходимости завершите только зарегистрированные процессы этого комплекта:

```bash
./course cleanup
./course down
```

Запишите одно наблюдение о commanded/measured motion и один вопрос. Практика формативная; домашняя работа выдаётся отдельно через `student-template`.

## Прямые команды подготовки и запуска

Этот вариант показывает действия, которые пока объединяет `course`, и готовит к самостоятельной работе с окружением. Используйте один способ запуска сцены одновременно.

Из `task05/` в **host terminal**:

```bash
docker compose up -d --build --wait
docker compose exec workspace bash
```

В **container shell**:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --event-handlers console_direct+
source /workspace/install/setup.bash
ros2 launch control_lab sim.launch.py mode:=practice
```

Для headless launch добавьте `gui:=false rviz:=false`. В каждом дополнительном shell повторяйте оба `source`. Ctrl+C нажимается в terminal, где работает launch; затем выйдите из shell через `exit`. В **host terminal** завершите только этот Compose project:

```bash
docker compose down
```

Interactive `docker compose exec` нужен для передачи Ctrl+C процессу внутри container. Скриптовый `exec -T` имеет другой режим сигналов и не заменяет этот учебный workflow. `docker compose down` завершает весь L05 environment при зависшем launch. Удаление wrapper не отменяет требований к bounded waits, hold и cleanup.
