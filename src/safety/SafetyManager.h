#pragma once

#include <stdint.h>

namespace gyrostab
{

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

} // namespace gyrostab
