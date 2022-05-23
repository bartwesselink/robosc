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
    "initial-step",                      /**< Initial step. */
    "delay-step",                        /**< Delay step. */
    "tau",                               /**< Tau step. */
    "message_correction.u_response",     /**< Event message_correction.u_response. */
    "message_no_line.u_response",        /**< Event message_no_line.u_response. */
    "message_stop.u_response",           /**< Event message_stop.u_response. */
    "message_continue.u_response",       /**< Event message_continue.u_response. */
    "message_move.c_trigger",    /**< Event message_move.c_trigger. */
    "data_move.c_pIELFJLWO5ADT", /**< Event data_move.c_pIELFJLWO5ADT. */
    "message_halt.c_trigger",            /**< Event message_halt.c_trigger. */
    "data_halt.c_pUOCHN7MH2YLV",         /**< Event data_halt.c_pUOCHN7MH2YLV. */
};

/** Enumeration names. */
const char *enum_names[] = {
    "awaiting",
    "data_p1POFUPHF3J9D",
    "data_pE12IJL1XOLG7",
    "in_service",
    "line_found",
    "no_line",
    "none",
    "stopped",
};

/* Constants. */


/* Functions. */


/* Input variables. */


/* State variables. */
controllerEnum component_EmergencyStop_; /**< Discrete variable "E component_EmergencyStop". */
controllerEnum component_LineDetector_;  /**< Discrete variable "E component_LineDetector". */
controllerEnum data_halt_;               /**< Discrete variable "E data_halt". */
controllerEnum data_move_;       /**< Discrete variable "E data_move". */

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
 * Execute code for event "data_halt.c_pUOCHN7MH2YLV".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent0(void) {
    BoolType guard = ((data_halt_) == (_controller_none)) || ((data_halt_) == (_controller_data_p1POFUPHF3J9D));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_pUOCHN7MH2YLV_, TRUE);
    #endif

    if ((data_halt_) == (_controller_none)) {
        data_halt_ = _controller_data_p1POFUPHF3J9D;
    } else if ((data_halt_) == (_controller_data_p1POFUPHF3J9D)) {
        data_halt_ = _controller_data_p1POFUPHF3J9D;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_pUOCHN7MH2YLV_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_move.c_pIELFJLWO5ADT".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent1(void) {
    BoolType guard = ((data_move_) == (_controller_none)) || ((data_move_) == (_controller_data_pE12IJL1XOLG7));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_pIELFJLWO5ADT_, TRUE);
    #endif

    if ((data_move_) == (_controller_none)) {
        data_move_ = _controller_data_pE12IJL1XOLG7;
    } else if ((data_move_) == (_controller_data_pE12IJL1XOLG7)) {
        data_move_ = _controller_data_pE12IJL1XOLG7;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_pIELFJLWO5ADT_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_continue.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent2(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_LineDetector_) == (_controller_awaiting)) || (((component_LineDetector_) == (_controller_line_found)) || ((component_LineDetector_) == (_controller_no_line))));
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
static BoolType execEvent3(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_LineDetector_) == (_controller_awaiting)) || (((component_LineDetector_) == (_controller_line_found)) || ((component_LineDetector_) == (_controller_no_line))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_correction_u_response_, TRUE);
    #endif

    if ((component_LineDetector_) == (_controller_awaiting)) {
        component_LineDetector_ = _controller_line_found;
    } else if ((component_LineDetector_) == (_controller_line_found)) {
        component_LineDetector_ = _controller_line_found;
    } else if ((component_LineDetector_) == (_controller_no_line)) {
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
static BoolType execEvent4(void) {
    BoolType guard = ((component_EmergencyStop_) == (_controller_stopped)) || ((component_LineDetector_) == (_controller_no_line));
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
static BoolType execEvent5(void) {
    BoolType guard = ((component_EmergencyStop_) == (_controller_in_service)) && ((component_LineDetector_) == (_controller_line_found));
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
static BoolType execEvent6(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_LineDetector_) == (_controller_awaiting)) || (((component_LineDetector_) == (_controller_line_found)) || ((component_LineDetector_) == (_controller_no_line))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_no_line_u_response_, TRUE);
    #endif

    if ((component_LineDetector_) == (_controller_awaiting)) {
        component_LineDetector_ = _controller_no_line;
    } else if ((component_LineDetector_) == (_controller_line_found)) {
        component_LineDetector_ = _controller_no_line;
    } else if ((component_LineDetector_) == (_controller_no_line)) {
        component_LineDetector_ = _controller_no_line;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_no_line_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_stop.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent7(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_LineDetector_) == (_controller_awaiting)) || (((component_LineDetector_) == (_controller_line_found)) || ((component_LineDetector_) == (_controller_no_line))));
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

        if (execEvent0()) continue;  /* (Try to) perform event "data_halt.c_pUOCHN7MH2YLV". */
        if (execEvent1()) continue;  /* (Try to) perform event "data_move.c_pIELFJLWO5ADT". */
        if (execEvent4()) continue;  /* (Try to) perform event "message_halt.c_trigger". */
        if (execEvent5()) continue;  /* (Try to) perform event "message_move.c_trigger". */
        break; /* No event fired, done with discrete steps. */
    }
}

/** First model call, initializing, and performing discrete events before the first time step. */
void controller_EngineFirstStep(void) {
    InitConstants();

    model_time = 0.0;

    component_EmergencyStop_ = _controller_in_service;
    component_LineDetector_ = _controller_awaiting;
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
        case data_halt_c_pUOCHN7MH2YLV_:
            return execEvent0();
        case data_move_c_pIELFJLWO5ADT_:
            return execEvent1();
        case message_continue_u_response_:
            return execEvent2();
        case message_correction_u_response_:
            return execEvent3();
        case message_halt_c_trigger_:
            return execEvent4();
        case message_move_c_trigger_:
            return execEvent5();
        case message_no_line_u_response_:
            return execEvent6();
        case message_stop_u_response_:
            return execEvent7();
        default:
            return FALSE;
    }
}
