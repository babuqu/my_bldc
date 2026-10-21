#ifndef PARAM_LIST_H
#define PARAM_LIST_H

/* (ID name, stable ID, value type, access, min, max, default, C type, field, optional publish statement) */
#define PARAM_LIST(X) \
    X(Target_SPEED,      0x0101u, PARAM_TYPE_INT32, PARAM_ACCESS_READ | PARAM_ACCESS_WRITE, -32760, 32760, 0,       int32_t, target_speed, ) \
    X(PARAM_ID_KP_SPEED, 0x0102u, PARAM_TYPE_FLOAT, PARAM_ACCESS_READ | PARAM_ACCESS_WRITE | PARAM_ACCESS_PERSIST, 0.0, 6.55, 0.5000f, float, kp, g_speed_pid.Proportion = state->kp;) \
    X(PARAM_ID_KI_SPEED, 0x0103u, PARAM_TYPE_FLOAT, PARAM_ACCESS_READ | PARAM_ACCESS_WRITE | PARAM_ACCESS_PERSIST, 0.0, 6.55, 0.25f,   float, ki, g_speed_pid.Integral = state->ki;) \
    X(PARAM_ID_KD_SPEED, 0x0104u, PARAM_TYPE_FLOAT, PARAM_ACCESS_READ | PARAM_ACCESS_WRITE | PARAM_ACCESS_PERSIST, 0.0, 6.55, 0.150f,  float, kd, g_speed_pid.Derivative = state->kd;)

#endif /* PARAM_LIST_H */
