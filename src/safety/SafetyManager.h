#pragma once

#include <stdint.h>

class Servo;

namespace gyrostab
{

struct ImuSample;
struct OrientationEstimate;

// Состояния жизненного цикла безопасности, описанные в архитектуре проекта.
enum class SafetyState : uint8_t
{
    Boot,
    Initializing,
    Calibrating,
    Ready,
    Active,
    Fault
};

// Флаги текущего этапа запуска и уровень запроса активации.
// В Ready он включает Active; в Active его снятие возвращает систему в Ready.
struct SafetyStateInputs
{
    bool initializationComplete;
    bool calibrationComplete;
    bool activationRequested;
};

// Обновляет один переход состояния за вызов.
// Fault является защёлкнутым состоянием; вход в него выполняет enterFault().
// При неизвестном значении SafetyState возвращает false и не меняет state.
bool updateSafetyState(SafetyState& state, const SafetyStateInputs& inputs);

// Разрешает выполнение контроллера только в состоянии Active.
// Функция не меняет состояние безопасности.
bool canRunController(SafetyState state);

// Возвращает true, если обязательные каналы IMU присутствуют и образец свежий.
// Проверка использует общую логику isSampleFresh() и не меняет образец или SafetyState.
bool checkSensorTimeout(const ImuSample& sample, uint32_t nowUs, uint32_t maximumAgeUs);

// Возвращает true, если оценка свежая и quaternion проходит проверку нормы.
// Не изменяет оценку, состояние безопасности или выход сервопривода.
bool checkEstimatorHealth(const OrientationEstimate& estimate, uint32_t nowUs,
                          uint32_t maximumAgeUs, float maximumNormDeviation);

// Проверяет, что целевой импульс уже находится в корректном диапазоне ESP32Servo и min/max.
// Не ограничивает исходную цель и не отправляет PWM.
bool checkCommandLimits(int targetPulseUs, int minimumPulseUs, int maximumPulseUs);

// Защёлкивает SafetyState::Fault; причину отказа сохраняет отдельная функция диагностики.
void enterFault(SafetyState& state);

// В FAULT повторно подаёт последний принятый импульс, не меняя положение и не отключая PWM.
// lastCommandPulseUs — последняя успешно отправленная команда; min/max — активные пределы.
// Возвращает false вне FAULT, при отключённой серве или недопустимой команде.
bool applySafeOutputPolicy(SafetyState state, ::Servo& servo,
                           int lastCommandPulseUs, int minimumPulseUs, int maximumPulseUs);

} // namespace gyrostab
