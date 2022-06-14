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
    "initial-step",                       /**< Initial step. */
    "delay-step",                         /**< Delay step. */
    "tau",                                /**< Tau step. */
    "message_scan_right.u_response",      /**< Event message_scan_right.u_response. */
    "data_scan_right.c_none",             /**< Event data_scan_right.c_none. */
    "message_scan_front.u_response",      /**< Event message_scan_front.u_response. */
    "data_scan_front.c_none",             /**< Event data_scan_front.c_none. */
    "message_scan_diag_right.u_response", /**< Event message_scan_diag_right.u_response. */
    "data_scan_diag_right.c_none",        /**< Event data_scan_diag_right.c_none. */
    "message_movement.c_trigger",         /**< Event message_movement.c_trigger. */
    "data_movement.c_none",               /**< Event data_movement.c_none. */
    "data_movement.c_pDZEQTIRGW9N1",      /**< Event data_movement.c_pDZEQTIRGW9N1. */
    "message_halt.c_trigger",             /**< Event message_halt.c_trigger. */
    "data_halt.c_none",                   /**< Event data_halt.c_none. */
    "data_halt.c_pYCT8ND9MZYDA",          /**< Event data_halt.c_pYCT8ND9MZYDA. */
    "message_turn_left.c_trigger",        /**< Event message_turn_left.c_trigger. */
    "data_turn_left.c_none",              /**< Event data_turn_left.c_none. */
    "data_turn_left.c_pGXOS0946TCCC",     /**< Event data_turn_left.c_pGXOS0946TCCC. */
    "message_turn_right.c_trigger",       /**< Event message_turn_right.c_trigger. */
    "data_turn_right.c_none",             /**< Event data_turn_right.c_none. */
    "data_turn_right.c_pWYRY9DSBD3QZ",    /**< Event data_turn_right.c_pWYRY9DSBD3QZ. */
    "message_rotate_done.u_response",     /**< Event message_rotate_done.u_response. */
    "data_rotate_done.c_none",            /**< Event data_rotate_done.c_none. */
    "message_stop.u_response",            /**< Event message_stop.u_response. */
    "data_stop.c_none",                   /**< Event data_stop.c_none. */
    "message_continue.u_response",        /**< Event message_continue.u_response. */
    "data_continue.c_none",               /**< Event data_continue.c_none. */
};

/** Enumeration names. */
const char *enum_names[] = {
    "data_p5BZ7N0434SVB",
    "data_p5XTILLQ2Z43J",
    "data_pJ5U3TUZ6JUE4",
    "data_pME01R4PQ72J6",
    "in_service",
    "no_wall_diag_right",
    "no_wall_front",
    "no_wall_right",
    "none",
    "ready",
    "stopped",
    "turning",
    "wall_diag_right",
    "wall_front",
    "wall_right",
};

/* Constants. */


/* Functions. */


/* Input variables. */
controllerEnum message_scan_right_i_response_;      /**< Input variable "E message_scan_right.i_response". */
controllerEnum message_scan_front_i_response_;      /**< Input variable "E message_scan_front.i_response". */
controllerEnum message_scan_diag_right_i_response_; /**< Input variable "E message_scan_diag_right.i_response". */

/* State variables. */
controllerEnum component_Distance_v_right_;      /**< Discrete variable "E component_Distance.v_right". */
controllerEnum component_Distance_v_front_;      /**< Discrete variable "E component_Distance.v_front". */
controllerEnum component_Distance_v_diag_right_; /**< Discrete variable "E component_Distance.v_diag_right". */
controllerEnum component_EmergencyStop_;         /**< Discrete variable "E component_EmergencyStop". */
controllerEnum component_Platform_;              /**< Discrete variable "E component_Platform". */
controllerEnum data_halt_;                       /**< Discrete variable "E data_halt". */
controllerEnum data_movement_;                   /**< Discrete variable "E data_movement". */
controllerEnum data_turn_left_;                  /**< Discrete variable "E data_turn_left". */
controllerEnum data_turn_right_;                 /**< Discrete variable "E data_turn_right". */

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
 * Execute code for event "data_continue.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent0(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_continue_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_continue_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_halt.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent1(void) {
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
 * Execute code for event "data_halt.c_pYCT8ND9MZYDA".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent2(void) {
    BoolType guard = ((data_halt_) == (_controller_none)) || ((data_halt_) == (_controller_data_pME01R4PQ72J6));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_pYCT8ND9MZYDA_, TRUE);
    #endif

    if ((data_halt_) == (_controller_none)) {
        data_halt_ = _controller_data_pME01R4PQ72J6;
    } else if ((data_halt_) == (_controller_data_pME01R4PQ72J6)) {
        data_halt_ = _controller_data_pME01R4PQ72J6;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_pYCT8ND9MZYDA_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_movement.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent3(void) {
    BoolType guard = FALSE;
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_movement_c_none_, TRUE);
    #endif

    if (FALSE) {
        data_movement_ = _controller_none;
    } else if (FALSE) {
        data_movement_ = _controller_none;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_movement_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_movement.c_pDZEQTIRGW9N1".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent4(void) {
    BoolType guard = ((data_movement_) == (_controller_none)) || ((data_movement_) == (_controller_data_pJ5U3TUZ6JUE4));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_movement_c_pDZEQTIRGW9N1_, TRUE);
    #endif

    if ((data_movement_) == (_controller_none)) {
        data_movement_ = _controller_data_pJ5U3TUZ6JUE4;
    } else if ((data_movement_) == (_controller_data_pJ5U3TUZ6JUE4)) {
        data_movement_ = _controller_data_pJ5U3TUZ6JUE4;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_movement_c_pDZEQTIRGW9N1_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_rotate_done.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent5(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_rotate_done_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_rotate_done_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_scan_diag_right.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent6(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_scan_diag_right_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_scan_diag_right_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_scan_front.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent7(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_scan_front_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_scan_front_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_scan_right.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent8(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_scan_right_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_scan_right_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_stop.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent9(void) {
    #if EVENT_OUTPUT
        controller_InfoEvent(data_stop_c_none_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(data_stop_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_turn_left.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent10(void) {
    BoolType guard = FALSE;
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_turn_left_c_none_, TRUE);
    #endif

    if (FALSE) {
        data_turn_left_ = _controller_none;
    } else if (FALSE) {
        data_turn_left_ = _controller_none;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_turn_left_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_turn_left.c_pGXOS0946TCCC".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent11(void) {
    BoolType guard = ((data_turn_left_) == (_controller_none)) || ((data_turn_left_) == (_controller_data_p5XTILLQ2Z43J));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_turn_left_c_pGXOS0946TCCC_, TRUE);
    #endif

    if ((data_turn_left_) == (_controller_none)) {
        data_turn_left_ = _controller_data_p5XTILLQ2Z43J;
    } else if ((data_turn_left_) == (_controller_data_p5XTILLQ2Z43J)) {
        data_turn_left_ = _controller_data_p5XTILLQ2Z43J;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_turn_left_c_pGXOS0946TCCC_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_turn_right.c_none".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent12(void) {
    BoolType guard = FALSE;
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_turn_right_c_none_, TRUE);
    #endif

    if (FALSE) {
        data_turn_right_ = _controller_none;
    } else if (FALSE) {
        data_turn_right_ = _controller_none;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_turn_right_c_none_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_turn_right.c_pWYRY9DSBD3QZ".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent13(void) {
    BoolType guard = ((data_turn_right_) == (_controller_none)) || ((data_turn_right_) == (_controller_data_p5BZ7N0434SVB));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_turn_right_c_pWYRY9DSBD3QZ_, TRUE);
    #endif

    if ((data_turn_right_) == (_controller_none)) {
        data_turn_right_ = _controller_data_p5BZ7N0434SVB;
    } else if ((data_turn_right_) == (_controller_data_p5BZ7N0434SVB)) {
        data_turn_right_ = _controller_data_p5BZ7N0434SVB;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_turn_right_c_pWYRY9DSBD3QZ_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_continue.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent14(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_Platform_) == (_controller_ready)) || ((component_Platform_) == (_controller_turning)));
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
 * Execute code for event "message_halt.c_trigger".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent15(void) {
    BoolType guard = (component_EmergencyStop_) == (_controller_stopped);
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
 * Execute code for event "message_movement.c_trigger".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent16(void) {
    BoolType guard = ((((component_Distance_v_right_) == (_controller_wall_right)) && ((component_Distance_v_front_) == (_controller_no_wall_front))) || ((((component_Distance_v_front_) == (_controller_no_wall_front)) && ((component_Distance_v_right_) == (_controller_no_wall_right))) && ((component_Distance_v_diag_right_) == (_controller_no_wall_diag_right)))) && (((component_Platform_) == (_controller_ready)) && ((component_EmergencyStop_ != _controller_stopped)));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_movement_c_trigger_, TRUE);
    #endif

    #if EVENT_OUTPUT
        controller_InfoEvent(message_movement_c_trigger_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_rotate_done.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent17(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_Platform_) == (_controller_ready)) || ((component_Platform_) == (_controller_turning)));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_rotate_done_u_response_, TRUE);
    #endif

    if ((component_Platform_) == (_controller_ready)) {
        component_Platform_ = _controller_ready;
    } else if ((component_Platform_) == (_controller_turning)) {
        component_Platform_ = _controller_ready;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_rotate_done_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_scan_diag_right.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent18(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_Platform_) == (_controller_ready)) || ((component_Platform_) == (_controller_turning)));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_scan_diag_right_u_response_, TRUE);
    #endif

    component_Distance_v_diag_right_ = message_scan_diag_right_i_response_;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_scan_diag_right_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_scan_front.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent19(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_Platform_) == (_controller_ready)) || ((component_Platform_) == (_controller_turning)));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_scan_front_u_response_, TRUE);
    #endif

    component_Distance_v_front_ = message_scan_front_i_response_;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_scan_front_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_scan_right.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent20(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_Platform_) == (_controller_ready)) || ((component_Platform_) == (_controller_turning)));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_scan_right_u_response_, TRUE);
    #endif

    component_Distance_v_right_ = message_scan_right_i_response_;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_scan_right_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_stop.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent21(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_Platform_) == (_controller_ready)) || ((component_Platform_) == (_controller_turning)));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_stop_u_response_, TRUE);
    #endif

    if ((component_EmergencyStop_) == (_controller_in_service)) {
        component_EmergencyStop_ = _controller_stopped;
    } else if ((component_EmergencyStop_) == (_controller_stopped)) {
        component_EmergencyStop_ = _controller_stopped;
    }
    if ((component_Platform_) == (_controller_ready)) {
        component_Platform_ = _controller_ready;
    } else if ((component_Platform_) == (_controller_turning)) {
        component_Platform_ = _controller_ready;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_stop_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_turn_left.c_trigger".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent22(void) {
    BoolType guard = ((component_Platform_) == (_controller_ready)) && (((component_Distance_v_front_) == (_controller_wall_front)) && (((component_Platform_) == (_controller_ready)) && ((component_EmergencyStop_ != _controller_stopped))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_turn_left_c_trigger_, TRUE);
    #endif

    component_Platform_ = _controller_turning;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_turn_left_c_trigger_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_turn_right.c_trigger".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent23(void) {
    BoolType guard = ((component_Platform_) == (_controller_ready)) && (((component_Distance_v_right_) == (_controller_no_wall_right)) && (((component_Distance_v_diag_right_) == (_controller_wall_diag_right)) && (((component_Platform_) == (_controller_ready)) && ((component_EmergencyStop_ != _controller_stopped)))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_turn_right_c_trigger_, TRUE);
    #endif

    component_Platform_ = _controller_turning;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_turn_right_c_trigger_, FALSE);
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

        if (execEvent0()) continue;  /* (Try to) perform event "data_continue.c_none". */
        if (execEvent1()) continue;  /* (Try to) perform event "data_halt.c_none". */
        if (execEvent2()) continue;  /* (Try to) perform event "data_halt.c_pYCT8ND9MZYDA". */
        if (execEvent3()) continue;  /* (Try to) perform event "data_movement.c_none". */
        if (execEvent4()) continue;  /* (Try to) perform event "data_movement.c_pDZEQTIRGW9N1". */
        if (execEvent5()) continue;  /* (Try to) perform event "data_rotate_done.c_none". */
        if (execEvent6()) continue;  /* (Try to) perform event "data_scan_diag_right.c_none". */
        if (execEvent7()) continue;  /* (Try to) perform event "data_scan_front.c_none". */
        if (execEvent8()) continue;  /* (Try to) perform event "data_scan_right.c_none". */
        if (execEvent9()) continue;  /* (Try to) perform event "data_stop.c_none". */
        if (execEvent10()) continue;  /* (Try to) perform event "data_turn_left.c_none". */
        if (execEvent11()) continue;  /* (Try to) perform event "data_turn_left.c_pGXOS0946TCCC". */
        if (execEvent12()) continue;  /* (Try to) perform event "data_turn_right.c_none". */
        if (execEvent13()) continue;  /* (Try to) perform event "data_turn_right.c_pWYRY9DSBD3QZ". */
        if (execEvent15()) continue;  /* (Try to) perform event "message_halt.c_trigger". */
        if (execEvent16()) continue;  /* (Try to) perform event "message_movement.c_trigger". */
        if (execEvent22()) continue;  /* (Try to) perform event "message_turn_left.c_trigger". */
        if (execEvent23()) continue;  /* (Try to) perform event "message_turn_right.c_trigger". */
        break; /* No event fired, done with discrete steps. */
    }
}

/** First model call, initializing, and performing discrete events before the first time step. */
void controller_EngineFirstStep(void) {
    InitConstants();

    model_time = 0.0;
    controller_AssignInputVariables();
    component_Distance_v_right_ = _controller_no_wall_right;
    component_Distance_v_front_ = _controller_no_wall_front;
    component_Distance_v_diag_right_ = _controller_no_wall_diag_right;
    component_EmergencyStop_ = _controller_in_service;
    component_Platform_ = _controller_ready;
    data_halt_ = _controller_none;
    data_movement_ = _controller_none;
    data_turn_left_ = _controller_none;
    data_turn_right_ = _controller_none;

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
        case data_continue_c_none_:
            return execEvent0();
        case data_halt_c_none_:
            return execEvent1();
        case data_halt_c_pYCT8ND9MZYDA_:
            return execEvent2();
        case data_movement_c_none_:
            return execEvent3();
        case data_movement_c_pDZEQTIRGW9N1_:
            return execEvent4();
        case data_rotate_done_c_none_:
            return execEvent5();
        case data_scan_diag_right_c_none_:
            return execEvent6();
        case data_scan_front_c_none_:
            return execEvent7();
        case data_scan_right_c_none_:
            return execEvent8();
        case data_stop_c_none_:
            return execEvent9();
        case data_turn_left_c_none_:
            return execEvent10();
        case data_turn_left_c_pGXOS0946TCCC_:
            return execEvent11();
        case data_turn_right_c_none_:
            return execEvent12();
        case data_turn_right_c_pWYRY9DSBD3QZ_:
            return execEvent13();
        case message_continue_u_response_:
            return execEvent14();
        case message_halt_c_trigger_:
            return execEvent15();
        case message_movement_c_trigger_:
            return execEvent16();
        case message_rotate_done_u_response_:
            return execEvent17();
        case message_scan_diag_right_u_response_:
            return execEvent18();
        case message_scan_front_u_response_:
            return execEvent19();
        case message_scan_right_u_response_:
            return execEvent20();
        case message_stop_u_response_:
            return execEvent21();
        case message_turn_left_c_trigger_:
            return execEvent22();
        case message_turn_right_c_trigger_:
            return execEvent23();
        default:
            return FALSE;
    }
}
