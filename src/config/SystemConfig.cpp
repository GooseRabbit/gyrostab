#include "SystemConfig.h"

namespace gyrostab
{

void loadDefaultConfig(SystemConfig& config)
{
    // Сначала создаём полностью обнулённый снимок: неподтверждённые
    // настройки остаются placeholders и не должны использоваться как рабочие.
    SystemConfig candidate = {};

    // Номинальная распиновка из текущего драйвера и архитектуры проекта.
    candidate.board.spiSckPin = 18;
    candidate.board.spiMisoPin = 19;
    candidate.board.spiMosiPin = 23;
    candidate.board.imuChipSelectPin = 5;
    candidate.board.servoSignalPin = 25;
    candidate.board.pinsVerified = false;

    // Только после полной подготовки заменяем переданную конфигурацию.
    config = candidate;
}

} // namespace gyrostab
