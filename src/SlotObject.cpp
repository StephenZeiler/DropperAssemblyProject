#include "SlotObject.h"
SlotObject::SlotObject(int id) : 
    slotId(id),
    hasError(false),
    currentPosition(0),
    capInjected(false),
    bulbInjected(false),
    pipetInjected(false)
{
    // Constructor - nothing else needed
}