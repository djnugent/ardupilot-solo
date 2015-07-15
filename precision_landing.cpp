/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Copter.h"
//
// functions to support precision landing
//

#if PRECISION_LANDING == ENABLED

void Copter::init_precland()
{
    copter.precland.init();
}

void Copter::update_precland()
{
    copter.precland.update(current_loc.alt);

    // log output
    Log_Write_Precland();
}

#endif
