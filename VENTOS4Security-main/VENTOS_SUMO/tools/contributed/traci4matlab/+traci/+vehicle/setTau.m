function setTau(vehID, tau)
%setTau Sets the driver's reaction time in s for this vehicle.
%   setTau(VEHID,TAU) Sets the driver's reaction time in s for this vehicle.

%   Copyright 2016 Universidad Nacional de Colombia,
%   Politecnico Jaime Isaza Cadavid.
%   Authors: Andres Acosta, Jairo Espinosa, Jorge Espinosa.
%   $Id: setTau.m 31 2016-09-28 15:16:56Z afacostag $

import traci.constants
traci.sendDoubleCmd(constants.CMD_SET_VEHICLE_VARIABLE, constants.VAR_TAU, vehID, tau);