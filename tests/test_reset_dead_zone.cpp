#include "../src/control/DeadZone.h"

#include <cassert>
#include <cstdio>

int main()
{
    // Сброс уже неактивного состояния безопасен.
    gyrostab::DeadZoneState state = {false};
    gyrostab::resetDeadZone(state);
    assert(!state.active);

    // Сброс активной dead-zone выключает её.
    state.active = true;
    gyrostab::resetDeadZone(state);
    assert(!state.active);

    // Повторный сброс остаётся идемпотентным.
    gyrostab::resetDeadZone(state);
    assert(!state.active);

    std::puts("resetDeadZone tests passed");
    return 0;
}
