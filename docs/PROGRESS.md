# Прогресс разработки Gyrostab

**Текущая ветка:** `main`  
**Состояние на:** 23 ноября 2026 года

## Общий показатель

**22 из 58 запланированных функций реализованы и добавлены в `main` — 37,9%.**

Процент считается по уникальным именам функций, перечисленным в `ARCHITECTURE.md` и `IMPLEMENTATION_PLAN.md` (совпадающие имена считаются один раз). `setup()` и `loop()`, структуры данных, документация, тестовые сценарии и аппаратные проверки в знаменатель не входят.

Функция считается реализованной, когда её код добавлен в `main` и блок принят. Это **не означает**, что прошивка уже собрана для целевой платы или проверена на реальном оборудовании.

## 1. Конфигурация и геометрия — 3/5

- [ ] `loadDefaultConfig()`
- [ ] `validateConfig()`
- [x] `normalizeVector3()`
- [x] `normalizeAxisConfig()`
- [x] `validateMountingConfig()`

## 2. Драйвер IMU и менеджер датчиков — 9/10

- [x] `beginImu()`
- [x] `configureImu()`
- [x] `readImuSample()`
- [x] `checkImuStatus()`
- [ ] `resetImuDataPath()`
- [x] `updateSensorData()`
- [x] `validateSensorSample()`
- [x] `isSampleFresh()`
- [x] `getLatestSample()`
- [x] `getSensorHealth()`

## 3. Калибровка — 4/4

- [x] `detectStationary()`
- [x] `calibrateGyroBias()`
- [x] `validateCalibration()`
- [x] `resetCalibration()`

## 4. Оценка ориентации — 3/5

- [x] `initializeEstimator()`
- [x] `updateOrientation()`
- [ ] `getOrientation()`
- [ ] `resetOrientation()`
- [x] `validateOrientation()`

## 5. Системы координат и опорная ориентация — 1/5

- [x] `transformImuToPlatform()`
- [ ] `captureReferenceOrientation()`
- [ ] `computeRelativeOrientation()`
- [ ] `projectErrorOnControlAxis()`
- [ ] `projectGyroRateOnControlAxis()`

## 6. Драйвер сервопривода — 0/6

- [ ] `beginServo()`
- [ ] `setServoTarget()`
- [ ] `limitServoPulse()`
- [ ] `limitServoSlewRate()`
- [ ] `writeServoPulse()`
- [ ] `disableServoOutput()`

## 7. Контроллер стабилизации — 0/7

- [ ] `computeControlError()`
- [ ] `computeDampingRate()`
- [ ] `computePdCorrection()`
- [ ] `resetController()`
- [ ] `validateControllerOutput()`
- [ ] `applyServoCommandLimits()`
- [ ] `runControlStep()`

## 8. Dead-zone с гистерезисом — 0/3

- [ ] `updateDeadZoneState()`
- [ ] `applyDeadZone()`
- [ ] `resetDeadZone()`

## 9. Безопасность и отказоустойчивость — 0/7

- [ ] `updateSafetyState()`
- [ ] `canRunController()`
- [ ] `checkSensorTimeout()`
- [ ] `checkEstimatorHealth()`
- [ ] `checkCommandLimits()`
- [ ] `enterFault()`
- [ ] `applySafeOutputPolicy()`

## 10. Магнитометр — 0/1

- [ ] `calibrateMagnetometer()`

## 11. Диагностика — 2/5

- [x] `publishRawImuTelemetry()`
- [x] `publishOrientationTelemetry()`
- [ ] `publishTelemetry()`
- [ ] `recordFault()`
- [ ] `updateCounters()`

## Сводка по этапам

| Этап | Готово | Всего |
|---|---:|---:|
| Конфигурация и геометрия | 3 | 5 |
| IMU и менеджер датчиков | 9 | 10 |
| Калибровка | 4 | 4 |
| Оценка ориентации | 3 | 5 |
| Координаты и опорная ориентация | 1 | 5 |
| Драйвер серво | 0 | 6 |
| Контроллер | 0 | 7 |
| Dead-zone | 0 | 3 |
| Безопасность | 0 | 7 |
| Магнитометр | 0 | 1 |
| Диагностика | 2 | 5 |
| **Итого** | **22** | **58** |

## Проверка и аппаратная готовность

- [x] Host-тесты геометрии: `normalizeVector3()`, `normalizeAxisConfig()`, `validateMountingConfig()`, `transformImuToPlatform()`.
- [x] Host-тесты `validateSensorSample()`: обязательные каналы, NaN/Inf, пределы выбранного full-scale, недопустимые диапазоны и необязательный магнитометр.
- [x] Host-тесты `isSampleFresh()`: точная граница, нулевой возраст, rollover `uint32_t`, будущая метка, неопубликованный образец и неоднозначный интервал.
- [x] Host-тесты `updateSensorData()`: новый валидный пакет сохраняется; отсутствие данных, ошибки чтения и невалидные обязательные каналы/диапазоны отклоняются; магнитометр остаётся необязательным.
- [x] Host-тесты `getLatestSample()`: пустой кэш, получение образца, свежесть на границе, отказ при устаревании, сохранение выхода при отказе, кэш после неудачного обновления, замена новым образцом и rollover.
- [x] Host-тесты `getSensorHealth()`: неизвестный статус и пустой кэш, валидный и устаревший образец, разделение признаков валидности/свежести, метаданные последнего образца и отдельная фиксация статуса драйвера.
- [x] Host-тесты `publishRawImuTelemetry()`: одна CSV-строка с 14 полями, фиксированный порядок, точность шесть знаков, флаги валидности и неизменность входного образца.
- [x] Host-тесты `publishOrientationTelemetry()`: одна CSV-строка с 7 полями, фиксированный порядок, точность шесть знаков, valid-флаг для невалидной оценки, крайнее значение `uint32_t` и неизменность входной оценки.
- [x] Host-тесты `validateMountingConfig()`: единичные и повёрнутые оси, допуск длины, ортогональность, правая ориентация, нулевые/неконечные векторы и некорректный допуск.
- [x] Host-тесты `transformImuToPlatform()`: identity и повёрнутые базисы, проекции в координаты платформы, NaN/Inf, переполнение float и сохранение входов/выхода при отказе.
- [x] Host-тесты `detectStationary()`: неподвижное окно, недостаточный размер/интервал, разрывы, движение по гироскопу и акселерометру, NaN/Inf, неверные пороги, порядок timestamp/sequence и rollover обоих счётчиков.
- [x] Host-тесты `calibrateGyroBias()`: среднее смещение по трём осям, отказ при движении/недостаточном окне/устаревании/NaN и неизменность выходного вектора при отказе.
- [x] Host-тесты `validateCalibration()`: допустимый bias, включительная граница отклонения, превышение предела и одиночный выброс, невалидный bias/порог, недостаточное и устаревшее окно, отказ при движении и неизменность входных данных.
- [x] Host-тесты `resetCalibration()`: обнуление всех трёх компонент bias, сброс признака валидности, идемпотентный повторный вызов и случай нулевого bias с ранее установленным флагом.
- [x] Host-тесты `initializeEstimator()`: полная перезапись состояния, единичный начальный кватернион, сброс timestamp/sequence, `valid == false` и повторная инициализация.
- [x] Host-тесты `updateOrientation()`: первый образец задаёт относительную привязку, точное интегрирование постоянной gyro-скорости, вычитание bias, норма quaternion, пропуск sequence при допустимом dt, rollover меток и отказные случаи без изменения состояния.
- [x] Host-тесты `validateOrientation()`: валидный unit quaternion, произвольное направление единичной нормы, точная граница допуска, превышение допуска, invalid flag, нулевой quaternion, NaN/Inf всех компонент, некорректный допуск и неизменность оценки.
- [x] Host-тесты драйвера с mock-интерфейсом SparkFun: проверены `beginImu()`, `configureImu()`, `readImuSample()`, `checkImuStatus()`, `validateSensorSample()`, `isSampleFresh()`, преобразование единиц и отказные случаи (включая SPI error, WrongID, NoData и восстановление OK).
- [x] Проверка драйвера с `-Wall -Wextra -Wpedantic -Werror`.
- [x] Повторная проверка драйвера с AddressSanitizer и UndefinedBehaviorSanitizer.
- [ ] Полная сборка Arduino Sketch для точного профиля ESP32 Nano.
- [ ] Проверка SPI и идентификатора ICM-20948 на реальной плате.
- [ ] Сверка фактической распиновки с проводкой.
- [ ] Чтение и проверка сырых данных акселерометра, гироскопа и магнитометра.
- [ ] Проверка ориентации, механики и привода на стенде.

## Правило обновления

После одобрения и добавления очередной функции в `main` её отметка здесь обновляется, затем пересчитывается общий показатель. Новые функции не добавляются в этот список без отдельного согласования. Текущий порядок разработки задаётся `IMPLEMENTATION_PLAN.md`; этот файл показывает только статус.
