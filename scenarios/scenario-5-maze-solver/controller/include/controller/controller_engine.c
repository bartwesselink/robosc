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
    "message_scan_front.u_response",      /**< Event message_scan_front.u_response. */
    "message_scan_diag_right.u_response", /**< Event message_scan_diag_right.u_response. */
    "message_movement.c_trigger",         /**< Event message_movement.c_trigger. */
    "data_movement.c_pDMBQ1Z3V0D7U",      /**< Event data_movement.c_pDMBQ1Z3V0D7U. */
    "message_halt.c_trigger",             /**< Event message_halt.c_trigger. */
    "data_halt.c_pYTRPAY1DIO5O",          /**< Event data_halt.c_pYTRPAY1DIO5O. */
    "message_turn_left.c_trigger",        /**< Event message_turn_left.c_trigger. */
    "data_turn_left.c_pBNDD3E1DBF59",     /**< Event data_turn_left.c_pBNDD3E1DBF59. */
    "message_turn_right.c_trigger",       /**< Event message_turn_right.c_trigger. */
    "data_turn_right.c_p9MM05JJ1R4JF",    /**< Event data_turn_right.c_p9MM05JJ1R4JF. */
    "message_rotate_done.u_response",     /**< Event message_rotate_done.u_response. */
    "message_stop.u_response",            /**< Event message_stop.u_response. */
    "message_continue.u_response",        /**< Event message_continue.u_response. */
};

/** Enumeration names. */
const char *enum_names[] = {
    "data_p0HO458J0GF7R",
    "data_pTRN2FIC8X6LY",
    "data_pWIUI30FAK9NC",
    "data_pXRJU2Q620EN1",
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
 * Execute code for event "data_halt.c_pYTRPAY1DIO5O".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent0(void) {
    BoolType guard = ((data_halt_) == (_controller_none)) || ((data_halt_) == (_controller_data_p0HO458J0GF7R));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_pYTRPAY1DIO5O_, TRUE);
    #endif

    if ((data_halt_) == (_controller_none)) {
        data_halt_ = _controller_data_p0HO458J0GF7R;
    } else if ((data_halt_) == (_controller_data_p0HO458J0GF7R)) {
        data_halt_ = _controller_data_p0HO458J0GF7R;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_pYTRPAY1DIO5O_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_movement.c_pDMBQ1Z3V0D7U".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent1(void) {
    BoolType guard = ((data_movement_) == (_controller_none)) || ((data_movement_) == (_controller_data_pWIUI30FAK9NC));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_movement_c_pDMBQ1Z3V0D7U_, TRUE);
    #endif

    if ((data_movement_) == (_controller_none)) {
        data_movement_ = _controller_data_pWIUI30FAK9NC;
    } else if ((data_movement_) == (_controller_data_pWIUI30FAK9NC)) {
        data_movement_ = _controller_data_pWIUI30FAK9NC;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_movement_c_pDMBQ1Z3V0D7U_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_turn_left.c_pBNDD3E1DBF59".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent2(void) {
    BoolType guard = ((data_turn_left_) == (_controller_none)) || ((data_turn_left_) == (_controller_data_pXRJU2Q620EN1));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_turn_left_c_pBNDD3E1DBF59_, TRUE);
    #endif

    if ((data_turn_left_) == (_controller_none)) {
        data_turn_left_ = _controller_data_pXRJU2Q620EN1;
    } else if ((data_turn_left_) == (_controller_data_pXRJU2Q620EN1)) {
        data_turn_left_ = _controller_data_pXRJU2Q620EN1;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_turn_left_c_pBNDD3E1DBF59_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_turn_right.c_p9MM05JJ1R4JF".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent3(void) {
    BoolType guard = ((data_turn_right_) == (_controller_none)) || ((data_turn_right_) == (_controller_data_pTRN2FIC8X6LY));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_turn_right_c_p9MM05JJ1R4JF_, TRUE);
    #endif

    if ((data_turn_right_) == (_controller_none)) {
        data_turn_right_ = _controller_data_pTRN2FIC8X6LY;
    } else if ((data_turn_right_) == (_controller_data_pTRN2FIC8X6LY)) {
        data_turn_right_ = _controller_data_pTRN2FIC8X6LY;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_turn_right_c_p9MM05JJ1R4JF_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_continue.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent4(void) {
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
static BoolType execEvent5(void) {
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
static BoolType execEvent6(void) {
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
static BoolType execEvent7(void) {
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
static BoolType execEvent8(void) {
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
static BoolType execEvent9(void) {
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
static BoolType execEvent10(void) {
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
static BoolType execEvent11(void) {
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
static BoolType execEvent12(void) {
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
static BoolType execEvent13(void) {
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

        if (execEvent0()) continue;  /* (Try to) perform event "data_halt.c_pYTRPAY1DIO5O". */
        if (execEvent1()) continue;  /* (Try to) perform event "data_movement.c_pDMBQ1Z3V0D7U". */
        if (execEvent2()) continue;  /* (Try to) perform event "data_turn_left.c_pBNDD3E1DBF59". */
        if (execEvent3()) continue;  /* (Try to) perform event "data_turn_right.c_p9MM05JJ1R4JF". */
        if (execEvent5()) continue;  /* (Try to) perform event "message_halt.c_trigger". */
        if (execEvent6()) continue;  /* (Try to) perform event "message_movement.c_trigger". */
        if (execEvent12()) continue;  /* (Try to) perform event "message_turn_left.c_trigger". */
        if (execEvent13()) continue;  /* (Try to) perform event "message_turn_right.c_trigger". */
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
        case data_halt_c_pYTRPAY1DIO5O_:
            return execEvent0();
        case data_movement_c_pDMBQ1Z3V0D7U_:
            return execEvent1();
        case data_turn_left_c_pBNDD3E1DBF59_:
            return execEvent2();
        case data_turn_right_c_p9MM05JJ1R4JF_:
            return execEvent3();
        case message_continue_u_response_:
            return execEvent4();
        case message_halt_c_trigger_:
            return execEvent5();
        case message_movement_c_trigger_:
            return execEvent6();
        case message_rotate_done_u_response_:
            return execEvent7();
        case message_scan_diag_right_u_response_:
            return execEvent8();
        case message_scan_front_u_response_:
            return execEvent9();
        case message_scan_right_u_response_:
            return execEvent10();
        case message_stop_u_response_:
            return execEvent11();
        case message_turn_left_c_trigger_:
            return execEvent12();
        case message_turn_right_c_trigger_:
            return execEvent13();
        default:
            return FALSE;
    }
}
