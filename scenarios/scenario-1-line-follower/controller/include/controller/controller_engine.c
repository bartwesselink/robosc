/* CIF to C translation of controller.cif
 * Generated file, DO NOT EDIT
 */

#include <stdio.h>
#include <stdlib.h>
#include "controller_engine.h"

#ifndef MAX_NUM_EVENTS
#define MAX_NUM_EVENTS 1000
#endif

/* What to do if a range error is found in an assignment? */
#ifdef KEEP_RUNNING
static inline void RangeErrorDetected(void) { /* Do nothing, error is already reported. */ }
#else
static inline void RangeErrorDetected(void) { exit(1); }
#endif

/* Type support code. */
int EnumTypePrint(controllerEnum value, char *dest, int start, int end) {
    int last = end - 1;
    const char *lit_name = enum_names[value];
    while (start < last && *lit_name) {
        dest[start++] = *lit_name;
        lit_name++;
    }
    dest[start] = '\0';
    return start;
}


/** Event names. */
const char *controller_event_names[] = {
    "initial-step",                          /**< Initial step. */
    "delay-step",                            /**< Delay step. */
    "tau",                                   /**< Tau step. */
    "message_correction.u_response",         /**< Event message_correction.u_response. */
    "data_correction.c_none",                /**< Event data_correction.c_none. */
    "message_no_line.u_response",            /**< Event message_no_line.u_response. */
    "data_no_line.c_none",                   /**< Event data_no_line.c_none. */
    "component_LidarSensor.c_pVF9ZLWJF9HHG", /**< Event component_LidarSensor.c_pVF9ZLWJF9HHG. */
    "component_LidarSensor.c_pIBXNH5R0L1DS", /**< Event component_LidarSensor.c_pIBXNH5R0L1DS. */
    "message_scan.u_response",               /**< Event message_scan.u_response. */
    "data_scan.c_none",                      /**< Event data_scan.c_none. */
    "message_stop.u_response",               /**< Event message_stop.u_response. */
    "data_stop.c_none",                      /**< Event data_stop.c_none. */
    "message_continue.u_response",           /**< Event message_continue.u_response. */
    "data_continue.c_none",                  /**< Event data_continue.c_none. */
    "message_move.c_trigger",                /**< Event message_move.c_trigger. */
    "data_move.c_none",                      /**< Event data_move.c_none. */
    "data_move.c_pLLL4DHRWZ0J5",             /**< Event data_move.c_pLLL4DHRWZ0J5. */
    "message_halt.c_trigger",                /**< Event message_halt.c_trigger. */
    "data_halt.c_none",                      /**< Event data_halt.c_none. */
    "data_halt.c_p6PLB4ODMJIWE",             /**< Event data_halt.c_p6PLB4ODMJIWE. */
};

/** Enumeration names. */
const char *enum_names[] = {
    "data_p8V5D37XULD3A",
    "data_pX6JOW6V7BDK7",
    "in_service",
    "line_found",
    "no_line",
    "none",
    "safe",
    "safe_distance",
    "stopped",
    "unsafe",
    "unsafe_distance",
};

/* Constants. */


/* Functions. */


/* Input variables. */
controllerEnum message_scan_i_response_; /**< Input variable "E message_scan.i_response". */

/* State variables. */
controllerEnum component_EmergencyStop_;                  /**< Discrete variable "E component_EmergencyStop". */
controllerEnum component_LidarSensor_v_current_distance_; /**< Discrete variable "E component_LidarSensor.v_current_distance". */
controllerEnum component_LidarSensor_;                    /**< Discrete variable "E component_LidarSensor". */
controllerEnum component_LineDetector_;                   /**< Discrete variable "E component_LineDetector". */
controllerEnum data_halt_;                                /**< Discrete variable "E data_halt". */
controllerEnum data_move_;                                /**< Discrete variable "E data_move". */

RealType model_time; /**< Current model time. */

/** Initialize constants. */
static void InitConstants(void) {

}

/** Print function. */
#if PRINT_OUTPUT
static void PrintOutput(controller_Event_ event, BoolType pre) {
}
#endif

/* Event execution code. */

/**
 * Execute code for event "component_LidarSensor.c_pIBXNH5R0L1DS".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent0(void) {
    BoolType guard = ((component_LidarSensor_) == (_controller_safe_distance)) && ((component_LidarSensor_v_current_distance_) == (_controller_unsafe));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(component_LidarSensor_c_pIBXNH5R0L1DS_, TRUE);
    #endif

    component_LidarSensor_ = _controller_unsafe_distance;

    #if EVENT_OUTPUT
        controller_InfoEvent(component_LidarSensor_c_pIBXNH5R0L1DS_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "component_LidarSensor.c_pVF9ZLWJF9HHG".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent1(void) {
    BoolType guard = ((component_LidarSensor_) == (_controller_unsafe_distance)) && ((component_LidarSensor_v_current_distance_) == (_controller_safe));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(component_LidarSensor_c_pVF9ZLWJF9HHG_, TRUE);
    #endif

    component_LidarSensor_ = _controller_safe_distance;

    #if EVENT_OUTPUT
        controller_InfoEvent(component_LidarSensor_c_pVF9ZLWJF9HHG_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_continue.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent2(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_continue_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_continue_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_correction.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent3(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_correction_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_correction_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_halt.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent4(void) {
    BoolType guard = FALSE;
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_none_, TRUE);
    #endif

    if (FALSE) {
        data_halt_ = _controller_none;
    } else if (FALSE) {
        data_halt_ = _controller_none;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_halt.c_p6PLB4ODMJIWE".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent5(void) {
    BoolType guard = ((data_halt_) == (_controller_none)) || ((data_halt_) == (_controller_data_p8V5D37XULD3A));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_p6PLB4ODMJIWE_, TRUE);
    #endif

    if ((data_halt_) == (_controller_none)) {
        data_halt_ = _controller_data_p8V5D37XULD3A;
    } else if ((data_halt_) == (_controller_data_p8V5D37XULD3A)) {
        data_halt_ = _controller_data_p8V5D37XULD3A;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_p6PLB4ODMJIWE_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_move.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent6(void) {
    BoolType guard = FALSE;
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_none_, TRUE);
    #endif

    if (FALSE) {
        data_move_ = _controller_none;
    } else if (FALSE) {
        data_move_ = _controller_none;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_move.c_pLLL4DHRWZ0J5".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent7(void) {
    BoolType guard = ((data_move_) == (_controller_none)) || ((data_move_) == (_controller_data_pX6JOW6V7BDK7));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_pLLL4DHRWZ0J5_, TRUE);
    #endif

    if ((data_move_) == (_controller_none)) {
        data_move_ = _controller_data_pX6JOW6V7BDK7;
    } else if ((data_move_) == (_controller_data_pX6JOW6V7BDK7)) {
        data_move_ = _controller_data_pX6JOW6V7BDK7;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_pLLL4DHRWZ0J5_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_no_line.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent8(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_no_line_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_no_line_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_scan.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent9(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_scan_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_scan_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_stop.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent10(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_stop_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_stop_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_continue.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent11(void) {
    BoolType guard = ((((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_LidarSensor_) == (_controller_unsafe_distance)) || ((component_LidarSensor_) == (_controller_safe_distance)))) && (((component_LineDetector_) == (_controller_no_line)) || ((component_LineDetector_) == (_controller_line_found)));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_continue_u_response_, TRUE);
    #endif

    if ((component_EmergencyStop_) == (_controller_in_service)) {
        component_EmergencyStop_ = _controller_in_service;
    } else if ((component_EmergencyStop_) == (_controller_stopped)) {
        component_EmergencyStop_ = _controller_in_service;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_continue_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_correction.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent12(void) {
    BoolType guard = ((((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_LidarSensor_) == (_controller_unsafe_distance)) || ((component_LidarSensor_) == (_controller_safe_distance)))) && (((component_LineDetector_) == (_controller_no_line)) || ((component_LineDetector_) == (_controller_line_found)));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_correction_u_response_, TRUE);
    #endif

    if ((component_LineDetector_) == (_controller_no_line)) {
        component_LineDetector_ = _controller_line_found;
    } else if ((component_LineDetector_) == (_controller_line_found)) {
        component_LineDetector_ = _controller_line_found;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_correction_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_halt.c_trigger".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent13(void) {
    BoolType guard = ((component_EmergencyStop_) == (_controller_stopped)) || (((component_LineDetector_) == (_controller_no_line)) || ((component_LidarSensor_) == (_controller_unsafe_distance)));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_halt_c_trigger_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(message_halt_c_trigger_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_move.c_trigger".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent14(void) {
    BoolType guard = ((component_EmergencyStop_) == (_controller_in_service)) && (((component_LineDetector_) == (_controller_line_found)) && ((component_LidarSensor_) == (_controller_safe_distance)));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_move_c_trigger_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(message_move_c_trigger_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_no_line.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent15(void) {
    BoolType guard = ((((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_LidarSensor_) == (_controller_unsafe_distance)) || ((component_LidarSensor_) == (_controller_safe_distance)))) && (((component_LineDetector_) == (_controller_no_line)) || ((component_LineDetector_) == (_controller_line_found)));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_no_line_u_response_, TRUE);
    #endif

    if ((component_LineDetector_) == (_controller_no_line)) {
        component_LineDetector_ = _controller_no_line;
    } else if ((component_LineDetector_) == (_controller_line_found)) {
        component_LineDetector_ = _controller_no_line;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_no_line_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_scan.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent16(void) {
    BoolType guard = ((((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_LidarSensor_) == (_controller_unsafe_distance)) || ((component_LidarSensor_) == (_controller_safe_distance)))) && (((component_LineDetector_) == (_controller_no_line)) || ((component_LineDetector_) == (_controller_line_found)));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_scan_u_response_, TRUE);
    #endif

    if ((component_LidarSensor_) == (_controller_unsafe_distance)) {
        component_LidarSensor_v_current_distance_ = message_scan_i_response_;
    } else if ((component_LidarSensor_) == (_controller_safe_distance)) {
        component_LidarSensor_v_current_distance_ = message_scan_i_response_;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_scan_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_stop.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent17(void) {
    BoolType guard = ((((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_LidarSensor_) == (_controller_unsafe_distance)) || ((component_LidarSensor_) == (_controller_safe_distance)))) && (((component_LineDetector_) == (_controller_no_line)) || ((component_LineDetector_) == (_controller_line_found)));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_stop_u_response_, TRUE);
    #endif

    if ((component_EmergencyStop_) == (_controller_in_service)) {
        component_EmergencyStop_ = _controller_stopped;
    } else if ((component_EmergencyStop_) == (_controller_stopped)) {
        component_EmergencyStop_ = _controller_stopped;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_stop_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Normalize and check the new value of a continuous variable after an update.
 * @param new_value Unnormalized new value of the continuous variable.
 * @param var_name Name of the continuous variable in the CIF model.
 * @return The normalized new value of the continuous variable.
 */
static inline RealType UpdateContValue(RealType new_value, const char *var_name) {
    if (isfinite(new_value)) {
        return (new_value == -0.0) ? 0.0 : new_value;
    }

    const char *err_type;
    if (isnan(new_value)) {
        err_type = "NaN";
    } else if (new_value > 0) {
        err_type = "+inf";
    } else {
        err_type = "-inf";
    }
    fprintf(stderr, "Continuous variable \"%s\" has become %s.\n", var_name, err_type);

#ifdef KEEP_RUNNING
    return 0.0;
#else
    exit(1);
#endif
}

/** Repeatedly perform discrete event steps, until no progress can be made any more. */
static void PerformEvents(void) {
    int count = 0;
    for (;;) {
        count++;
        if (count > MAX_NUM_EVENTS) { /* 'Infinite' loop detection. */
            fprintf(stderr, "Warning: Quitting after performing %d events, infinite loop?\n", count);
            break;
        }

        if (execEvent0()) continue;  /* (Try to) perform event "component_LidarSensor.c_pIBXNH5R0L1DS". */
        if (execEvent1()) continue;  /* (Try to) perform event "component_LidarSensor.c_pVF9ZLWJF9HHG". */
        if (execEvent2()) continue;  /* (Try to) perform event "data_continue.c_none". */
        if (execEvent3()) continue;  /* (Try to) perform event "data_correction.c_none". */
        if (execEvent4()) continue;  /* (Try to) perform event "data_halt.c_none". */
        if (execEvent5()) continue;  /* (Try to) perform event "data_halt.c_p6PLB4ODMJIWE". */
        if (execEvent6()) continue;  /* (Try to) perform event "data_move.c_none". */
        if (execEvent7()) continue;  /* (Try to) perform event "data_move.c_pLLL4DHRWZ0J5". */
        if (execEvent8()) continue;  /* (Try to) perform event "data_no_line.c_none". */
        if (execEvent9()) continue;  /* (Try to) perform event "data_scan.c_none". */
        if (execEvent10()) continue;  /* (Try to) perform event "data_stop.c_none". */
        if (execEvent13()) continue;  /* (Try to) perform event "message_halt.c_trigger". */
        if (execEvent14()) continue;  /* (Try to) perform event "message_move.c_trigger". */
        break; /* No event fired, done with discrete steps. */
    }
}

/** First model call, initializing, and performing discrete events before the first time step. */
void controller_EngineFirstStep(void) {
    InitConstants();

    model_time = 0.0;
    controller_AssignInputVariables();
    component_EmergencyStop_ = _controller_in_service;
    component_LidarSensor_v_current_distance_ = _controller_unsafe;
    component_LidarSensor_ = _controller_unsafe_distance;
    component_LineDetector_ = _controller_no_line;
    data_halt_ = _controller_none;
    data_move_ = _controller_none;

    #if PRINT_OUTPUT
        /* pre-initial and post-initial prints. */
        PrintOutput(EVT_INITIAL_, TRUE);
        PrintOutput(EVT_INITIAL_, FALSE);
    #endif

    PerformEvents();

    #if PRINT_OUTPUT
        /* pre-timestep print. */
        PrintOutput(EVT_DELAY_, TRUE);
    #endif
}

/**
 * Engine takes a time step of length \a delta.
 * @param delta Length of the time step.
 */
void controller_EngineTimeStep(double delta) {
    controller_AssignInputVariables();

    /* Update continuous variables. */
    if (delta > 0.0) {

        model_time += delta;
    }

    #if PRINT_OUTPUT
        /* post-timestep print. */
        PrintOutput(EVT_DELAY_, FALSE);
    #endif

    PerformEvents();

    #if PRINT_OUTPUT
        /* pre-timestep print. */
        PrintOutput(EVT_DELAY_, TRUE);
    #endif
}

/**
  * Engine (attempts) to perform a single event.
  * @param event The event to perform.
  * @return Whether the specified event could be executed.
  */
BoolType controller_EnginePerformEvent(controller_Event_ event) {
    switch (event) {
        case component_LidarSensor_c_pIBXNH5R0L1DS_:
            return execEvent0();
        case component_LidarSensor_c_pVF9ZLWJF9HHG_:
            return execEvent1();
        case data_continue_c_none_:
            return execEvent2();
        case data_correction_c_none_:
            return execEvent3();
        case data_halt_c_none_:
            return execEvent4();
        case data_halt_c_p6PLB4ODMJIWE_:
            return execEvent5();
        case data_move_c_none_:
            return execEvent6();
        case data_move_c_pLLL4DHRWZ0J5_:
            return execEvent7();
        case data_no_line_c_none_:
            return execEvent8();
        case data_scan_c_none_:
            return execEvent9();
        case data_stop_c_none_:
            return execEvent10();
        case message_continue_u_response_:
            return execEvent11();
        case message_correction_u_response_:
            return execEvent12();
        case message_halt_c_trigger_:
            return execEvent13();
        case message_move_c_trigger_:
            return execEvent14();
        case message_no_line_u_response_:
            return execEvent15();
        case message_scan_u_response_:
            return execEvent16();
        case message_stop_u_response_:
            return execEvent17();
        default:
            return FALSE;
    }
}
