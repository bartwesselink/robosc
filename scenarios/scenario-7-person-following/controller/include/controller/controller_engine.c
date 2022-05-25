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
    "message_scan.u_response",           /**< Event message_scan.u_response. */
    "message_bounding_boxes.u_response", /**< Event message_bounding_boxes.u_response. */
    "message_stop.u_response",           /**< Event message_stop.u_response. */
    "message_continue.u_response",       /**< Event message_continue.u_response. */
    "message_move.c_trigger",            /**< Event message_move.c_trigger. */
    "data_move.c_p0LY3EZ9AGQNL",         /**< Event data_move.c_p0LY3EZ9AGQNL. */
    "data_move.c_pUAQU3XG1WYHR",         /**< Event data_move.c_pUAQU3XG1WYHR. */
    "data_move.c_pUSGSRSK1JLC3",         /**< Event data_move.c_pUSGSRSK1JLC3. */
    "message_halt.c_trigger",            /**< Event message_halt.c_trigger. */
    "data_halt.c_pWJL900TG040Y",         /**< Event data_halt.c_pWJL900TG040Y. */
};

/** Enumeration names. */
const char *enum_names[] = {
    "data_p5J0MYKIFPJAO",
    "data_pC7OJ0RYRXLPS",
    "data_pIGGZFZSZR4F9",
    "data_pSMSKNS3BMH48",
    "detected",
    "free",
    "in_service",
    "initializing",
    "none",
    "person",
    "stopped",
};

/* Constants. */


/* Functions. */


/* Input variables. */
controllerEnum message_scan_i_response_; /**< Input variable "E message_scan.i_response". */

/* State variables. */
controllerEnum component_EmergencyStop_;      /**< Discrete variable "E component_EmergencyStop". */
controllerEnum component_Scanner_v_distance_; /**< Discrete variable "E component_Scanner.v_distance". */
controllerEnum component_YoloxDetection_;     /**< Discrete variable "E component_YoloxDetection". */
controllerEnum data_halt_;                    /**< Discrete variable "E data_halt". */
controllerEnum data_move_;                    /**< Discrete variable "E data_move". */

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
 * Execute code for event "data_halt.c_pWJL900TG040Y".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent0(void) {
    BoolType guard = ((data_halt_) == (_controller_none)) || ((data_halt_) == (_controller_data_pSMSKNS3BMH48));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_pWJL900TG040Y_, TRUE);
    #endif

    if ((data_halt_) == (_controller_none)) {
        data_halt_ = _controller_data_pSMSKNS3BMH48;
    } else if ((data_halt_) == (_controller_data_pSMSKNS3BMH48)) {
        data_halt_ = _controller_data_pSMSKNS3BMH48;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_halt_c_pWJL900TG040Y_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_move.c_p0LY3EZ9AGQNL".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent1(void) {
    BoolType guard = ((((data_move_) == (_controller_none)) && (((component_YoloxDetection_) == (_controller_detected)) && ((component_Scanner_v_distance_) == (_controller_person)))) || (((data_move_) == (_controller_data_pIGGZFZSZR4F9)) && (((component_YoloxDetection_) == (_controller_detected)) && ((component_Scanner_v_distance_) == (_controller_person))))) || ((((data_move_) == (_controller_data_p5J0MYKIFPJAO)) && (((component_YoloxDetection_) == (_controller_detected)) && ((component_Scanner_v_distance_) == (_controller_person)))) || (((data_move_) == (_controller_data_pC7OJ0RYRXLPS)) && (((component_YoloxDetection_) == (_controller_detected)) && ((component_Scanner_v_distance_) == (_controller_person)))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_p0LY3EZ9AGQNL_, TRUE);
    #endif

    if (((data_move_) == (_controller_none)) && (((component_YoloxDetection_) == (_controller_detected)) && ((component_Scanner_v_distance_) == (_controller_person)))) {
        data_move_ = _controller_data_pIGGZFZSZR4F9;
    } else if (((data_move_) == (_controller_data_pIGGZFZSZR4F9)) && (((component_YoloxDetection_) == (_controller_detected)) && ((component_Scanner_v_distance_) == (_controller_person)))) {
        data_move_ = _controller_data_pIGGZFZSZR4F9;
    } else if (((data_move_) == (_controller_data_p5J0MYKIFPJAO)) && (((component_YoloxDetection_) == (_controller_detected)) && ((component_Scanner_v_distance_) == (_controller_person)))) {
        data_move_ = _controller_data_pIGGZFZSZR4F9;
    } else if (((data_move_) == (_controller_data_pC7OJ0RYRXLPS)) && (((component_YoloxDetection_) == (_controller_detected)) && ((component_Scanner_v_distance_) == (_controller_person)))) {
        data_move_ = _controller_data_pIGGZFZSZR4F9;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_p0LY3EZ9AGQNL_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_move.c_pUAQU3XG1WYHR".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent2(void) {
    BoolType guard = ((((data_move_) == (_controller_none)) && (((component_YoloxDetection_) == (_controller_detected)) && ((component_Scanner_v_distance_) == (_controller_free)))) || (((data_move_) == (_controller_data_pIGGZFZSZR4F9)) && (((component_YoloxDetection_) == (_controller_detected)) && ((component_Scanner_v_distance_) == (_controller_free))))) || ((((data_move_) == (_controller_data_p5J0MYKIFPJAO)) && (((component_YoloxDetection_) == (_controller_detected)) && ((component_Scanner_v_distance_) == (_controller_free)))) || (((data_move_) == (_controller_data_pC7OJ0RYRXLPS)) && (((component_YoloxDetection_) == (_controller_detected)) && ((component_Scanner_v_distance_) == (_controller_free)))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_pUAQU3XG1WYHR_, TRUE);
    #endif

    if (((data_move_) == (_controller_none)) && (((component_YoloxDetection_) == (_controller_detected)) && ((component_Scanner_v_distance_) == (_controller_free)))) {
        data_move_ = _controller_data_p5J0MYKIFPJAO;
    } else if (((data_move_) == (_controller_data_pIGGZFZSZR4F9)) && (((component_YoloxDetection_) == (_controller_detected)) && ((component_Scanner_v_distance_) == (_controller_free)))) {
        data_move_ = _controller_data_p5J0MYKIFPJAO;
    } else if (((data_move_) == (_controller_data_p5J0MYKIFPJAO)) && (((component_YoloxDetection_) == (_controller_detected)) && ((component_Scanner_v_distance_) == (_controller_free)))) {
        data_move_ = _controller_data_p5J0MYKIFPJAO;
    } else if (((data_move_) == (_controller_data_pC7OJ0RYRXLPS)) && (((component_YoloxDetection_) == (_controller_detected)) && ((component_Scanner_v_distance_) == (_controller_free)))) {
        data_move_ = _controller_data_p5J0MYKIFPJAO;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_pUAQU3XG1WYHR_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "data_move.c_pUSGSRSK1JLC3".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent3(void) {
    BoolType guard = ((((data_move_) == (_controller_none)) && ((component_YoloxDetection_) == (_controller_initializing))) || (((data_move_) == (_controller_data_pIGGZFZSZR4F9)) && ((component_YoloxDetection_) == (_controller_initializing)))) || ((((data_move_) == (_controller_data_p5J0MYKIFPJAO)) && ((component_YoloxDetection_) == (_controller_initializing))) || (((data_move_) == (_controller_data_pC7OJ0RYRXLPS)) && ((component_YoloxDetection_) == (_controller_initializing))));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_pUSGSRSK1JLC3_, TRUE);
    #endif

    if (((data_move_) == (_controller_none)) && ((component_YoloxDetection_) == (_controller_initializing))) {
        data_move_ = _controller_data_pC7OJ0RYRXLPS;
    } else if (((data_move_) == (_controller_data_pIGGZFZSZR4F9)) && ((component_YoloxDetection_) == (_controller_initializing))) {
        data_move_ = _controller_data_pC7OJ0RYRXLPS;
    } else if (((data_move_) == (_controller_data_p5J0MYKIFPJAO)) && ((component_YoloxDetection_) == (_controller_initializing))) {
        data_move_ = _controller_data_pC7OJ0RYRXLPS;
    } else if (((data_move_) == (_controller_data_pC7OJ0RYRXLPS)) && ((component_YoloxDetection_) == (_controller_initializing))) {
        data_move_ = _controller_data_pC7OJ0RYRXLPS;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(data_move_c_pUSGSRSK1JLC3_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_bounding_boxes.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent4(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_YoloxDetection_) == (_controller_initializing)) || ((component_YoloxDetection_) == (_controller_detected)));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_bounding_boxes_u_response_, TRUE);
    #endif

    if ((component_YoloxDetection_) == (_controller_initializing)) {
        component_YoloxDetection_ = _controller_detected;
    } else if ((component_YoloxDetection_) == (_controller_detected)) {
        component_YoloxDetection_ = _controller_detected;
    }

    #if EVENT_OUTPUT
        controller_InfoEvent(message_bounding_boxes_u_response_, FALSE);
    #endif
    return TRUE;
}

/**
 * Execute code for event "message_continue.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent5(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_YoloxDetection_) == (_controller_initializing)) || ((component_YoloxDetection_) == (_controller_detected)));
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
static BoolType execEvent6(void) {
    BoolType guard = ((component_EmergencyStop_) == (_controller_stopped)) || ((component_Scanner_v_distance_) == (_controller_person));
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
static BoolType execEvent7(void) {
    BoolType guard = ((component_EmergencyStop_) == (_controller_in_service)) && ((component_YoloxDetection_) == (_controller_detected));
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
 * Execute code for event "message_scan.u_response".
 *
 * @return Whether the event was performed.
 */
static BoolType execEvent8(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_YoloxDetection_) == (_controller_initializing)) || ((component_YoloxDetection_) == (_controller_detected)));
    if (!guard) return FALSE;

    #if EVENT_OUTPUT
        controller_InfoEvent(message_scan_u_response_, TRUE);
    #endif

    component_Scanner_v_distance_ = message_scan_i_response_;

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
static BoolType execEvent9(void) {
    BoolType guard = (((component_EmergencyStop_) == (_controller_in_service)) || ((component_EmergencyStop_) == (_controller_stopped))) && (((component_YoloxDetection_) == (_controller_initializing)) || ((component_YoloxDetection_) == (_controller_detected)));
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

        if (execEvent0()) continue;  /* (Try to) perform event "data_halt.c_pWJL900TG040Y". */
        if (execEvent1()) continue;  /* (Try to) perform event "data_move.c_p0LY3EZ9AGQNL". */
        if (execEvent2()) continue;  /* (Try to) perform event "data_move.c_pUAQU3XG1WYHR". */
        if (execEvent3()) continue;  /* (Try to) perform event "data_move.c_pUSGSRSK1JLC3". */
        if (execEvent6()) continue;  /* (Try to) perform event "message_halt.c_trigger". */
        if (execEvent7()) continue;  /* (Try to) perform event "message_move.c_trigger". */
        break; /* No event fired, done with discrete steps. */
    }
}

/** First model call, initializing, and performing discrete events before the first time step. */
void controller_EngineFirstStep(void) {
    InitConstants();

    model_time = 0.0;
    controller_AssignInputVariables();
    component_EmergencyStop_ = _controller_in_service;
    component_Scanner_v_distance_ = _controller_free;
    component_YoloxDetection_ = _controller_initializing;
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
        case data_halt_c_pWJL900TG040Y_:
            return execEvent0();
        case data_move_c_p0LY3EZ9AGQNL_:
            return execEvent1();
        case data_move_c_pUAQU3XG1WYHR_:
            return execEvent2();
        case data_move_c_pUSGSRSK1JLC3_:
            return execEvent3();
        case message_bounding_boxes_u_response_:
            return execEvent4();
        case message_continue_u_response_:
            return execEvent5();
        case message_halt_c_trigger_:
            return execEvent6();
        case message_move_c_trigger_:
            return execEvent7();
        case message_scan_u_response_:
            return execEvent8();
        case message_stop_u_response_:
            return execEvent9();
        default:
            return FALSE;
    }
}
