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

// Флаги текущего этапа запуска и одноразовый запрос активации.
// activationRequested обрабатывается только когда текущее состояние Ready.
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

} // namespace gyrostab
