#include "../src/safety/SafetyManager.h"

#include <cassert>
#include <cstdio>
#include <cstdint>

int main()
{
    using gyrostab::SafetyState;
    using gyrostab::canRunController;

    assert(!canRunController(SafetyState::Boot));
    assert(!canRunController(SafetyState::Initializing));
    assert(!canRunController(SafetyState::Calibrating));
    assert(!canRunController(SafetyState::Ready));
    assert(canRunController(SafetyState::Active));
    assert(!canRunController(SafetyState::Fault));

    // Неизвестные значения enum также не разрешают запуск.
    assert(!canRunController(static_cast<SafetyState>(UINT8_MAX)));

    // Передача enum по значению не меняет исходное значение.
    const SafetyState state = SafetyState::Active;
    assert(canRunController(state));
    assert(state == SafetyState::Active);

    std::puts("canRunController tests passed");
    return 0;
}
