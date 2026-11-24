#include "../src/control/ControlStep.h"

#include <cassert>
#include <cstdio>

int main()
{
    // Сброс уже пустого состояния остаётся безопасным.
    gyrostab::ControllerState state = {};
    gyrostab::resetController(state);
    assert(!state.hasProcessedSample);
    assert(state.lastProcessedSampleSequence == 0U);

    // Сброс ранее обработанного образца удаляет признак и sequence.
    state.hasProcessedSample = true;
    state.lastProcessedSampleSequence = 12345U;
    gyrostab::resetController(state);
    assert(!state.hasProcessedSample);
    assert(state.lastProcessedSampleSequence == 0U);

    // Повторный сброс идемпотентен.
    gyrostab::resetController(state);
    assert(!state.hasProcessedSample);
    assert(state.lastProcessedSampleSequence == 0U);

    std::puts("resetController tests passed");
    return 0;
}
