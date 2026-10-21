#ifndef PARAM_MODBUS_LIST_H
#define PARAM_MODBUS_LIST_H

/* (zero-based holding register, parameter ID, scale, wire type) */
#define PARAM_MODBUS_LIST(X) \
    X(0u, Target_SPEED,      1.0,     PARAM_MODBUS_S16) \
    X(1u, PARAM_ID_KP_SPEED, 10000.0, PARAM_MODBUS_U16) \
    X(2u, PARAM_ID_KI_SPEED, 10000.0, PARAM_MODBUS_U16) \
    X(3u, PARAM_ID_KD_SPEED, 10000.0, PARAM_MODBUS_U16)

#endif /* PARAM_MODBUS_LIST_H */
