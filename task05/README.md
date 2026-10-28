# L05 Практика: управление суставами и ROS controllers

ROS 2 Jazzy, Gazebo Harmonic и стандартные ros2_control controllers. На занятии используем готовый 18 DoF гуманоид с закреплённым тазом. Практика начинается с одного сустава и обратной связи, затем знакомит с ROS graph, controllers и interfaces. Результат — standing/neutral posture → wave → neutral и отмена движения с измеренной остановкой.

- [Практика с преподавателем](docs/Practice_L05.md).
- [Статический пример wave](examples/practice_wave_goal.yaml): полное ROS message, которое разбираем вместе.

## Подготовка и запуск

Нужны Docker и Compose v2. Команды выполняются из папки `task05/` репозитория `tasks`. Образ собирается локально; установка ROS в систему не требуется. Первая загрузка packages может занять время, поэтому подготовьте окружение до занятия.

На Linux и WSL перед запуском проверьте UID/GID в host terminal:

```bash
id -u
id -g
```

Если любое число отличается от `1000`, вручную добавьте в игнорируемый Git файл `.env` внутри `task05/` строки ниже, подставив фактические числа из вывода. Если `.env` уже существует, измените только эти строки и сохраните остальные настройки:

```dotenv
LOCAL_UID=1001
LOCAL_GID=1001
```

На macOS с Docker Desktop используйте значения Compose по умолчанию.

```bash
./course up
./course build
./course doctor
./course desktop
./course practice
```

Откройте ссылку, напечатанную `./course desktop`, в браузере: там доступны Gazebo и RViz. Оставьте terminal с `./course practice` работающим. В новом terminal из той же папки проверьте controllers и отправьте первую цель:

```bash
./course control list
./course control interfaces
./course control goal neutral --duration 3
./course control goal reach --duration 3
```

Для работы без окон используйте `./course practice headless`. Не запускайте обе сцены одновременно.

Полные шаги с записью measured state, wave и отменой приведены в [руководстве практики](docs/Practice_L05.md). В его конце показан тот же запуск явными Docker Compose, `colcon`, `source` и ROS 2 командами. Дополнительные shells нужны для одновременных измерений и trajectories.

## Проверки и завершение

```bash
./course test
```

Это проверки рабочего runtime и модели практики. Live поведение проверяется отдельно по измерениям в Gazebo.

Остановите foreground launch сочетанием Ctrl+C в его terminal. При необходимости завершите зарегистрированные процессы L05, затем окружение:

```bash
./course cleanup
./course down
```

Домашняя работа выдаётся отдельно через репозиторий `student-template`; этот комплект содержит рабочую практику.
